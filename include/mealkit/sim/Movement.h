#pragma once

#include "mealkit/core/DetMath.h"
#include "mealkit/core/Reflect.h"
#include "mealkit/physics/PhysicsWorld.h"
#include "mealkit/sim/Ragdoll.h"

#include <algorithm>
#include <cmath>

// Character movement modes (docs/CODING_MODEL.md "Character movement"). Not inheritance: each Pawn kind picks a mode + MoveTuning.
// Input/AI → MoveIntent → move() (per mode) → physics step → senseMover() (position, velocity, grounded).
// Mover is simulation state (a field of the game's Pawn, so it's hashed and snapshotted); the physics-side state is in PhysicsWorld.
//   Kinematic: Jolt CharacterVirtual. Precise control, stairs, slopes, moving platforms. Pushes boxes, isn't pushed back by them.
//   Capsule:   rigid capsule with locked rotation, velocity steered toward the wish. Shoving party games; stairs/slopes feel awkward.
//   Floating:  rigid capsule hovering on a spring over a downward ray. Physics interaction + smooth over bumps and small steps.
//   Ragdoll:   active ragdoll (sim/Ragdoll.h). The pelvis hovers and steers like Floating and is turned upright toward the move
//              direction; the other parts are motorized toward a gait pose. knockRagdoll() makes it limp for a while (slapstick).
// Call move() for every Mover in a fixed order (Pawn id order) before PhysicsWorld::step, and senseMover() after it.
namespace mealkit::sim {

MEALKIT_ENUM(MoveMode, u8, Kinematic, Capsule, Floating, Ragdoll);

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
    MEALKIT_FIELD(f32, ragdollHover, 0.03f, "Ragdoll: gap kept under the feet (m)");
    MEALKIT_FIELD(f32, ragdollTorque, 600.0f, "Ragdoll: motor torque (N·m, the chest gets about this much)");
    MEALKIT_FIELD(f32, ragdollLimp, 0.03f, "Ragdoll: motor torque while knocked down (fraction)");
    MEALKIT_FIELD(f32, ragdollFrequency, 8.0f, "Ragdoll: motor stiffness (Hz)");
    MEALKIT_FIELD(f32, ragdollUpright, 10.0f, "Ragdoll: turning upright and toward the move direction (1/s)");
    MEALKIT_FIELD(f32, ragdollStride, 0.6f, "Ragdoll: leg swing (radians)");
    MEALKIT_FIELD(f32, ragdollCadence, 4.0f, "Ragdoll: gait phase per meter walked (radians/m)");
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
    u32 group = 0; // Ragdoll: nonzero collision group unique to this character (its parts ignore each other)
};

MEALKIT_STRUCT(Mover) {
    MEALKIT_FIELD(MoveMode, mode, MoveMode::Kinematic, "Mode");
    MEALKIT_FIELD(physics::BodyId, body, physics::kInvalidBody, "Body (Kinematic: the body following the character)");
    MEALKIT_FIELD(physics::CharacterId, character, physics::kInvalidCharacter, "Character (Kinematic)");
    MEALKIT_FIELD(f32, halfHeight, 0.0f, "Half height (m)");
    MEALKIT_FIELD(u32, grounded, 0, "Grounded");
    MEALKIT_FIELD(f32, groundGap, 0.0f, "Floating/Ragdoll: distance from the capsule bottom to the ground (m, -1 if none in reach)");
    MEALKIT_FIELD(Ragdoll, ragdoll, {}, "Ragdoll: parts and joints");
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
    if (mode == MoveMode::Ragdoll) {
        const Vec3 feet = shape.position - Vec3{0.0f, shape.halfHeight, 0.0f};
        m.ragdoll = createRagdoll(world, feet, shape.halfHeight * 2.0f, shape.mass, shape.group, t.ragdollTorque,
                                  t.ragdollFrequency);
        m.body = m.ragdoll.parts[0]; // the pelvis
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
    if (m.mode == MoveMode::Ragdoll) destroyRagdoll(world, m.ragdoll);
    else if (m.character != physics::kInvalidCharacter) world.destroyCharacter(m.character); // also its body
    else if (m.body != physics::kInvalidBody) world.destroy(m.body);
    m.character = physics::kInvalidCharacter;
    m.body = physics::kInvalidBody;
}

inline Vec3 moverVelocity(const physics::PhysicsWorld& world, const Mover& m) {
    return m.mode == MoveMode::Kinematic ? world.characterVelocity(m.character) : world.linearVelocity(m.body);
}

// Ragdoll: the whole body gets the pelvis's velocity change (limbs come along)
inline void addRagdollVelocity(physics::PhysicsWorld& world, const Ragdoll& r, Vec3 delta) {
    for (u32 i = 0; i < kRagdollParts; ++i) world.addLinearVelocity(r.parts[i], delta);
}

inline void setMoverVelocity(physics::PhysicsWorld& world, const Mover& m, Vec3 v) {
    if (m.mode == MoveMode::Kinematic) world.setCharacterVelocity(m.character, v);
    else if (m.mode == MoveMode::Ragdoll) addRagdollVelocity(world, m.ragdoll, v - world.linearVelocity(m.body));
    else world.setLinearVelocity(m.body, v);
}

// Ragdoll: goes limp for ticks (motors weak, no balance or hovering) and gets a velocity change. Other modes just get the velocity
inline void knockRagdoll(physics::PhysicsWorld& world, Mover& m, u32 ticks, Vec3 delta) {
    if (m.mode != MoveMode::Ragdoll) {
        setMoverVelocity(world, m, moverVelocity(world, m) + delta);
        return;
    }
    m.ragdoll.limpTicks = ticks > m.ragdoll.limpTicks ? ticks : m.ragdoll.limpTicks;
    addRagdollVelocity(world, m.ragdoll, delta);
}

// Ragdoll pelvis: angular velocity that turns it upright and toward yaw (arithmetic only: quaternion error × rate)
inline Vec3 ragdollUprightVelocity(Quat pelvis, Vec3 wish, f32 rate) {
    f32 yaw;
    if (lengthSq(wish) > 1e-4f) yaw = detAtan2(wish.x, wish.z); // const-ok: no direction below this
    else {
        const Vec3 forward = rotate(pelvis, {0.0f, 0.0f, 1.0f});
        yaw = detAtan2(forward.x, forward.z);
    }
    Quat error = detYawQuat(yaw) * Quat{-pelvis.x, -pelvis.y, -pelvis.z, pelvis.w};
    if (error.w < 0.0f) error = {-error.x, -error.y, -error.z, -error.w};
    return Vec3{error.x, error.y, error.z} * (2.0f * rate);
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
    case MoveMode::Ragdoll: {
        Ragdoll& r = m.ragdoll;
        if (r.limpTicks > 0) { // knocked down: physics only, no jumping
            --r.limpTicks;
            driveRagdoll(world, r, RagdollPose{}, t.ragdollTorque * t.ragdollLimp);
            return false;
        }
        if (jumped) v.y = t.jumpSpeed;
        else if (m.groundGap > -1.0f) // ground in reach (lying down too: the spring gets it back up)
            v.y += ((t.ragdollHover - m.groundGap) * t.floatSpring - v.y * t.floatDamping - gravity.y) * dt;
        world.setLinearVelocity(m.body, v);
        // The rest of the body: weightless while held up (the pelvis spring carries it), and it jumps along
        const Vec3 lift = jumped ? Vec3{0.0f, t.jumpSpeed, 0.0f} : m.groundGap > -1.0f ? gravity * -dt : Vec3{};
        for (u32 i = 1; i < kRagdollParts; ++i) world.addLinearVelocity(r.parts[i], lift);
        world.setAngularVelocity(m.body, ragdollUprightVelocity(world.rotation(m.body), wish, t.ragdollUpright));
        const f32 moving = std::sqrt(v.x * v.x + v.z * v.z);
        r.phase += moving * dt * t.ragdollCadence;
        if (r.phase > kDetPi) r.phase -= 2.0f * kDetPi; // keep it in -pi..pi
        const f32 amount = m.grounded && speed > 0.0f ? std::min(1.0f, moving / speed) : 0.0f;
        driveRagdoll(world, r, ragdollGait(r.phase, amount, t.ragdollStride), t.ragdollTorque);
        break;
    }
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
    if (m.mode == MoveMode::Ragdoll) { // position = the standing capsule center above the feet under the pelvis
        Ragdoll& r = m.ragdoll;
        senseRagdoll(world, r);
        const Vec3 pelvis = r.partPositions[0];
        const f32 lift = ragdollPelvisHeight(r);
        position = pelvis + Vec3{0.0f, m.halfHeight - lift, 0.0f};
        velocity = world.linearVelocity(m.body);
        const f32 reach = lift + t.groundProbe + t.ragdollHover * 2.0f + m.halfHeight * 0.5f; // also senses a fall early
        physics::RayHit hit;
        const bool hitGround = world.castRay(pelvis, {0.0f, -reach, 0.0f}, hit, m.body, r.group);
        m.groundGap = hitGround ? std::max(hit.fraction * reach - lift, -0.9f * lift) : -1.0f; // const-ok: lying stays above -1 (none)
        m.grounded = hitGround && r.limpTicks == 0 && m.groundGap <= t.ragdollHover + t.groundProbe ? 1u : 0u;
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
