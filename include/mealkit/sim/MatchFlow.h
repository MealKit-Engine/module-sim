#pragma once

#include "mealkit/core/Reflect.h"
#include "mealkit/net/Lockstep.h"
#include "mealkit/sim/TickTimer.h"

#include <array>
#include <span>

// Match flow (docs/CODING_MODEL.md "Match flow"): waiting → countdown → playing → round end → (countdown ...) → results.
// MatchFlow is simulation state (a field of the game's WorldState, so it's hashed and snapshotted), MatchRules come from tuning.
// advanceMatch() is called once per tick and returns the transition made on that tick; the game's sim/Match.cpp switches on it
// and calls its own functions (onRoundStart, ...) directly. No callbacks. Transitions depend only on tick numbers and the players passed in.
namespace mealkit::sim {

MEALKIT_ENUM(FlowPhase, u8, Waiting, Countdown, Playing, RoundEnd, Results);

inline constexpr u32 kNoSide = 0xFFFFFFFFu; // no winner (draw, or nobody)

// Rules (read from the game's tuning at match start). 0 turns a rule off.
struct MatchRules {
    u32 minPlayers = 1;    // Waiting until this many players are present
    u32 countdown = 0;     // ticks before each round
    u32 roundLength = 0;   // ticks; 0 = no time limit
    u32 roundEndDelay = 0; // ticks the round result is shown
    u32 rounds = 1;        // best of N: the match ends when a side has more than half of the rounds, or after N rounds
    u32 scoreLimit = 0;    // a side reaching this round score wins the round
    bool elimination = false; // the round ends when at most one side has players alive
    u32 teamCount = 0;     // 0 or 1 = free for all (each player is a side); else side = slot % teamCount
    bool coop = false;     // every player is side 0 against the game: time running out or everyone out (elimination) is a loss
                           // (roundWinner kNoSide), and the game decides a win itself with finishRound(..., 0)
};

// One player as seen by the flow this tick (built by the game from its PlayerSlot scoreboard; not state)
struct MatchPlayer {
    bool present = false; // connected (absent players count for nothing)
    bool alive = true;    // still in the round (elimination)
    u32 score = 0;        // round score
};

MEALKIT_STRUCT(MatchFlow) {
    MEALKIT_FIELD(FlowPhase, phase, FlowPhase::Waiting, "Phase");
    MEALKIT_FIELD(TickTimer, phaseEnd, {}, "Phase end tick (countdown, round time, round end)");
    MEALKIT_FIELD(u32, round, 0, "Round (first is 1)");
    MEALKIT_FIELD(u32, roundWinner, kNoSide, "Last round winner (side)");
    MEALKIT_FIELD(u32, matchWinner, kNoSide, "Match winner (side)");
    MEALKIT_ARRAY(u32, net::kMaxPlayers, roundWins, "Rounds won per side");
};

enum class FlowEvent : u8 {
    None,
    CountdownStart, // phase became Countdown (round + 1 coming)
    RoundStart,     // phase became Playing: reset round state (scores, lives, positions)
    RoundEnd,       // phase became RoundEnd: flow.roundWinner is set
    MatchEnd,       // phase became Results: flow.matchWinner is set
};

inline u32 matchSide(const MatchRules& rules, u32 slot) {
    return rules.coop ? 0u : rules.teamCount > 1 ? slot % rules.teamCount : slot;
}
inline u32 matchSideCount(const MatchRules& rules) {
    return rules.coop ? 1u : rules.teamCount > 1 ? rules.teamCount : net::kMaxPlayers;
}

// The side with the single highest value among present sides (kNoSide on a tie or if none)
inline u32 bestSide(const std::array<u32, net::kMaxPlayers>& value, const std::array<bool, net::kMaxPlayers>& present, u32 sides) {
    u32 best = kNoSide;
    bool tie = false;
    for (u32 i = 0; i < sides; ++i) {
        if (!present[i]) continue;
        if (best == kNoSide || value[i] > value[best]) best = i, tie = false;
        else if (value[i] == value[best]) tie = true;
    }
    return tie ? kNoSide : best;
}

// Starts the flow at match start (World::reset). Returns the first transition (Countdown or straight into a round).
inline FlowEvent startMatchFlow(MatchFlow& flow, const MatchRules& rules, u32 tick, std::span<const MatchPlayer> players);

// Ends the round now with a winner the game decided (kNoSide = nobody), e.g. a co-op team reaching the goal. Only while Playing
// (returns None otherwise); call it from the game's step like the other transitions and handle the returned RoundEnd.
inline FlowEvent finishRound(MatchFlow& flow, const MatchRules& rules, u32 tick, u32 winner) {
    if (flow.phase != FlowPhase::Playing) return FlowEvent::None;
    flow.phase = FlowPhase::RoundEnd;
    flow.roundWinner = winner < matchSideCount(rules) ? winner : kNoSide;
    if (flow.roundWinner != kNoSide) ++flow.roundWins[flow.roundWinner];
    flow.phaseEnd.start(tick, rules.roundEndDelay);
    return FlowEvent::RoundEnd;
}

// One tick of the flow. Call at the start of the game's step (after commands).
inline FlowEvent advanceMatch(MatchFlow& flow, const MatchRules& rules, u32 tick, std::span<const MatchPlayer> players) {
    const u32 sides = matchSideCount(rules);
    std::array<u32, net::kMaxPlayers> score{};
    std::array<bool, net::kMaxPlayers> present{}, alive{};
    u32 presentPlayers = 0;
    for (u32 slot = 0; slot < players.size() && slot < net::kMaxPlayers; ++slot) {
        const MatchPlayer& p = players[slot];
        if (!p.present) continue;
        const u32 side = matchSide(rules, slot);
        ++presentPlayers;
        present[side] = true;
        score[side] += p.score;
        alive[side] = alive[side] || p.alive;
    }

    auto beginCountdown = [&]() {
        flow.phase = FlowPhase::Countdown;
        flow.phaseEnd.start(tick, rules.countdown);
        return FlowEvent::CountdownStart;
    };
    auto beginRound = [&]() {
        flow.phase = FlowPhase::Playing;
        ++flow.round;
        flow.roundWinner = kNoSide;
        flow.phaseEnd.start(tick, rules.roundLength);
        return FlowEvent::RoundStart;
    };

    switch (flow.phase) {
    case FlowPhase::Waiting:
        if (presentPlayers < (rules.minPlayers ? rules.minPlayers : 1u)) return FlowEvent::None;
        if (rules.countdown == 0) return beginRound();
        return beginCountdown();
    case FlowPhase::Countdown:
        return flow.phaseEnd.ready(tick) ? beginRound() : FlowEvent::None;
    case FlowPhase::Playing: {
        u32 presentSides = 0, aliveSides = 0, lastAlive = kNoSide;
        for (u32 i = 0; i < sides; ++i) {
            presentSides += present[i] ? 1u : 0u;
            if (present[i] && alive[i]) ++aliveSides, lastAlive = i;
        }
        bool over = false;
        u32 winner = kNoSide;
        if (rules.scoreLimit > 0) {
            const u32 best = bestSide(score, present, sides);
            if (best != kNoSide && score[best] >= rules.scoreLimit) over = true, winner = best;
        }
        if (!over && rules.elimination && aliveSides <= (presentSides > 1 ? 1u : 0u)) over = true, winner = lastAlive;
        if (!over && rules.roundLength > 0 && flow.phaseEnd.ready(tick))
            over = true, winner = rules.coop ? kNoSide : bestSide(score, present, sides);
        if (!over) return FlowEvent::None;
        return finishRound(flow, rules, tick, winner);
    }
    case FlowPhase::RoundEnd: {
        if (!flow.phaseEnd.ready(tick)) return FlowEvent::None;
        const u32 rounds = rules.rounds ? rules.rounds : 1u;
        u32 most = 0;
        for (u32 i = 0; i < sides; ++i) most = flow.roundWins[i] > most ? flow.roundWins[i] : most;
        if (flow.round < rounds && most * 2 <= rounds) return beginCountdown();
        std::array<bool, net::kMaxPlayers> any{};
        for (u32 i = 0; i < sides; ++i) any[i] = present[i] || flow.roundWins[i] > 0;
        flow.phase = FlowPhase::Results;
        // Co-op: the team wins the match with more rounds won than lost
        flow.matchWinner = rules.coop ? (flow.roundWins[0] * 2 > flow.round ? 0u : kNoSide) : bestSide(flow.roundWins, any, sides);
        return FlowEvent::MatchEnd;
    }
    case FlowPhase::Results: return FlowEvent::None;
    }
    return FlowEvent::None;
}

inline FlowEvent startMatchFlow(MatchFlow& flow, const MatchRules& rules, u32 tick, std::span<const MatchPlayer> players) {
    flow = MatchFlow{};
    return advanceMatch(flow, rules, tick, players);
}

} // namespace mealkit::sim
