#pragma once

#include "mealkit/core/DetMath.h"
#include "mealkit/core/Reflect.h"
#include "mealkit/physics/PhysicsWorld.h"

#include <algorithm>
#include <cmath>

// Character movement modes (docs/CODING_MODEL.md "Character movement"). Not inheritance: each Pawn kind picks a mode + MoveTuning.
// Input/AI → MoveIntent → move() (per mode) → physics step → senseMover() (position, velocity, grounded).
// Mover is simulation state (a field of the game's Pawn, so it's hashed and snapshotted); the physics-side state is in PhysicsWorld.
//   Kinematic: Jolt CharacterVirtual. Precise control, stairs, slopes, moving platforms. Pushes boxes, isn't pushed back by them.
//   Capsule:   rigid capsule with locked rotation, velocity steered toward the wish. Shoving party games; stairs/slopes feel awkward.
//   Floating:  rigid capsule hovering on a spring over a downward ray. Physics interaction + smooth over bumps and small steps.
// Call move() for every Mover in a fixed order (Pawn id order) before PhysicsWorld::step, and senseMover() after it.
namespace mealkit::sim {

MEALKIT_ENUM(MoveMode, u8, Kinematic, Capsule, Floating);

MEALKIT_STRUCT(MoveTuning) {
    MEALKIT_FIELD(f32, speed, 7.0f, "Speed (m/s)");
    MEALKIT_FIELD(f32, groundAccel, 12.0f, "Ground acceleration (1/s)");
    MEALKIT_FIELD(f32, airAccel, 3.0f, "Air acceleration (1/s)");
    MEALKIT_FIELD(f32, jumpSpeed, 8.0f, "Jump speed (m/s)");
    MEALKIT_FIELD(f32, jumpMaxRise, 1.0f, "No jump while rising faster than this (m/s)");
    MEALKIT_FIELD(f32, groundProbe, 0.12f, "Ground detection margin (m)");
    MEALKIT_FIELD(f32, maxSlope, 50.0f, "Kinematic: steepest walkable slope (degrees)");
    MEALKIT_FIELD(f32, stepUp, 0.4f, "Kinematic: step height (m)");
    MEALKIT_FIELD(f32, stickDown, 0.5f, "Kinematic: snap to floor below (m)");
    MEALKIT_FIELD(f32, floatHeight, 0.3f, "Floating: hover gap under the capsule (m)");
    MEALKIT_FIELD(f32, floatSpring, 300.0f, "Floating: spring (1/s²)");
    MEALKIT_FIELD(f32, floatDamping, 25.0f, "Floating: damping (1/s)");
};

// What a character wants to do this tick (humans from input, AI from decisions). Not state.
struct MoveIntent {
    Vec3 wish; // horizontal move direction, length 0..1 (longer is clamped)
    bool jump = false;
};

struct MoverShape {
    Vec3 position;        // capsule center
    f32 halfHeight = 0.9f; // half of the whole height
    f32 radius = 0.45f;
    f32 mass = 80.0f;
};

MEALKIT_STRUCT(Mover) {
    MEALKIT_FIELD(MoveMode, mode, MoveMode::Kinematic, "Mode");
    MEALKIT_FIELD(physics::BodyId, body, physics::kInvalidBody, "Body (Kinematic: the body following the character)");
    MEALKIT_FIELD(physics::CharacterId, character, physics::kInvalidCharacter, "Character (Kinematic)");
    MEALKIT_FIELD(f32, halfHeight, 0.0f, "Half height (m)");
    MEALKIT_FIELD(u32, grounded, 0, "Grounded");
    MEALKIT_FIELD(f32, groundGap, 0.0f, "Floating: distance from the capsule bottom to the ground (m, -1 if none in reach)");
};

inline Mover createMover(physics::PhysicsWorld& world, MoveMode mode, const MoverShape& shape, const MoveTuning& t) {
    Mover m;
    m.mode = mode;
    m.halfHeight = shape.halfHeight;
    const f32 cylinder = std::max(shape.halfHeight - shape.radius, 0.01f);
    if (mode == MoveMode::Kinematic) {
        physics::CharacterDesc d;
        d.position = shape.position;
        d.halfHeight = cylinder;
        d.radius = shape.radius;
        d.mass = shape.mass;
        d.maxSlopeCos = detCos(t.maxSlope * (kDetPi / 180.0f)); // const-ok: degrees → radians
        m.character = world.createCharacter(d);
        m.body = world.characterBody(m.character);
        return m;
    }
    physics::BodyDesc d;
    d.position = shape.position;
    d.lockRotation = true;
    d.friction = 0.0f;
    d.restitution = 0.0f;
    d.linearDamping = 0.0f;
    d.mass = shape.mass;
    d.allowSleep = false;
    m.body = world.createCapsule(cylinder, shape.radius, d);
    return m;
}

inline void destroyMover(physics::PhysicsWorld& world, Mover& m) {
    if (m.character != physics::kInvalidCharacter) world.destroyCharacter(m.character); // also its body
    else if (m.body != physics::kInvalidBody) world.destroy(m.body);
    m.character = physics::kInvalidCharacter;
    m.body = physics::kInvalidBody;
}

inline Vec3 moverVelocity(const physics::PhysicsWorld& world, const Mover& m) {
    return m.mode == MoveMode::Kinematic ? world.characterVelocity(m.character) : world.linearVelocity(m.body);
}

inline void setMoverVelocity(physics::PhysicsWorld& world, const Mover& m, Vec3 v) {
    if (m.mode == MoveMode::Kinematic) world.setCharacterVelocity(m.character, v);
    else world.setLinearVelocity(m.body, v);
}

// Applies this tick's intent. Returns true when it jumped (for view notifications).
inline bool move(physics::PhysicsWorld& world, Mover& m, const MoveIntent& intent, const MoveTuning& t, f32 speedScale,
                 Vec3 gravity, f32 dt) {
    Vec3 wish = intent.wish;
    const f32 wishLen2 = lengthSq(wish);
    if (wishLen2 > 1.0f) wish = wish / std::sqrt(wishLen2);
    const f32 speed = t.speed * speedScale;

    Vec3 v = moverVelocity(world, m);
    const f32 accel = std::min(1.0f, (m.grounded ? t.groundAccel : t.airAccel) * dt);
    v.x += (wish.x * speed - v.x) * accel;
    v.z += (wish.z * speed - v.z) * accel;
    const bool jumped = m.grounded && intent.jump && v.y < t.jumpMaxRise;

    switch (m.mode) {
    case MoveMode::Kinematic: {
        // Not a rigid body: gravity is ours. On the ground, ride along with what's underfoot.
        const physics::CharacterGround ground = world.characterGround(m.character);
        if (ground.supported && v.y <= ground.velocity.y) v.y = ground.velocity.y;
        else v = v + gravity * dt;
        if (jumped) v.y = t.jumpSpeed;
        world.moveCharacter(m.character, v, gravity, dt, t.stepUp, t.stickDown);
        break;
    }
    case MoveMode::Capsule:
        if (jumped) v.y = t.jumpSpeed;
        world.setLinearVelocity(m.body, v);
        break;
    case MoveMode::Floating:
        if (jumped) v.y = t.jumpSpeed;
        else if (m.groundGap >= 0.0f) // ground within reach: spring toward the hover gap, damped (cancels gravity at rest)
            v.y += ((t.floatHeight - m.groundGap) * t.floatSpring - v.y * t.floatDamping - gravity.y) * dt;
        world.setLinearVelocity(m.body, v);
        break;
    }
    return jumped;
}

// After the physics step: position, velocity, grounded
inline void senseMover(const physics::PhysicsWorld& world, Mover& m, const MoveTuning& t, Vec3& position, Vec3& velocity) {
    if (m.mode == MoveMode::Kinematic) {
        position = world.characterPosition(m.character);
        velocity = world.characterVelocity(m.character);
        m.grounded = world.characterGround(m.character).supported ? 1u : 0u;
        return;
    }
    position = world.position(m.body);
    velocity = world.linearVelocity(m.body);
    // Floating looks further down: the spring holds it floatHeight above the ground
    const f32 reach = m.halfHeight + t.groundProbe + (m.mode == MoveMode::Floating ? t.floatHeight * 2.0f : 0.0f);
    physics::RayHit hit;
    const bool hitGround = world.castRay(position, {0.0f, -reach, 0.0f}, hit, m.body);
    m.groundGap = hitGround ? hit.fraction * reach - m.halfHeight : -1.0f;
    m.grounded = hitGround && (m.mode != MoveMode::Floating || m.groundGap <= t.floatHeight + t.groundProbe) ? 1u : 0u;
}

} // namespace mealkit::sim
