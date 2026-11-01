#pragma once

#include "mealkit/core/DetMath.h"
#include "mealkit/core/Reflect.h"
#include "mealkit/physics/PhysicsWorld.h"

#include <array>

// Active ragdoll body (the Ragdoll movement mode in mealkit/sim/Movement.h, docs/CODING_MODEL.md "Character movement").
// One fixed humanoid of 11 parts sized from the character height (not from a model's skeleton), 10 motorized joints.
// Parts are created in RagdollPart order and joints in child order, so every peer gets the same body and joint ids.
// Target poses are per-joint rotations relative to the parent part (RagdollPose). The Ragdoll mode uses the procedural gait
// here (ragdollGait); a game may fill RagdollPose from asset samplePoseTick itself (simulation pose sampling, DECISIONS 2026-10-15).
namespace mealkit::sim {

MEALKIT_ENUM(RagdollPart, u8, Pelvis, Chest, Head, UpperArmL, LowerArmL, UpperArmR, LowerArmR, ThighL, ShinL, ThighR, ShinR);
inline constexpr u32 kRagdollParts = 11;
inline constexpr u32 kRagdollJoints = kRagdollParts - 1; // joint i drives part i + 1 (the pelvis is the root)

// Parent part per part (the pelvis has none)
inline constexpr std::array<u8, kRagdollParts> kRagdollParent = {0, 0, 1, 1, 3, 1, 5, 0, 7, 0, 9};

// Shape and placement of one part in the rest pose (standing, arms down), relative to the feet, for a 1 m tall body.
// Boxes (pelvis, chest), a sphere (head, half.x = radius), capsules for limbs (half.x = radius, half.y = half the cylinder).
enum class RagdollShape : u8 { Box, Sphere, Capsule };
struct RagdollPartLayout {
    RagdollShape shape;
    Vec3 center; // rest center above the feet (fractions of the height)
    Vec3 half;
    Vec3 pivot;  // joint to the parent (fractions of the height, unused for the pelvis)
    f32 mass;    // fraction of the total mass
};

// const-ok: body proportions (fractions of the height and mass), roughly human
inline constexpr std::array<RagdollPartLayout, kRagdollParts> kRagdollLayout = {{
    {RagdollShape::Box, {0.0f, 0.55f, 0.0f}, {0.09f, 0.05f, 0.06f}, {}, 0.26f},
    {RagdollShape::Box, {0.0f, 0.70f, 0.0f}, {0.10f, 0.10f, 0.06f}, {0.0f, 0.60f, 0.0f}, 0.27f},
    {RagdollShape::Sphere, {0.0f, 0.875f, 0.0f}, {0.065f, 0.0f, 0.0f}, {0.0f, 0.80f, 0.0f}, 0.08f},
    {RagdollShape::Capsule, {0.135f, 0.70f, 0.0f}, {0.03f, 0.05f, 0.0f}, {0.135f, 0.78f, 0.0f}, 0.03f},
    {RagdollShape::Capsule, {0.135f, 0.54f, 0.0f}, {0.027f, 0.05f, 0.0f}, {0.135f, 0.62f, 0.0f}, 0.02f},
    {RagdollShape::Capsule, {-0.135f, 0.70f, 0.0f}, {0.03f, 0.05f, 0.0f}, {-0.135f, 0.78f, 0.0f}, 0.03f},
    {RagdollShape::Capsule, {-0.135f, 0.54f, 0.0f}, {0.027f, 0.05f, 0.0f}, {-0.135f, 0.62f, 0.0f}, 0.02f},
    {RagdollShape::Capsule, {0.055f, 0.375f, 0.0f}, {0.045f, 0.08f, 0.0f}, {0.055f, 0.50f, 0.0f}, 0.10f},
    {RagdollShape::Capsule, {0.055f, 0.13f, 0.0f}, {0.04f, 0.085f, 0.0f}, {0.055f, 0.25f, 0.0f}, 0.045f},
    {RagdollShape::Capsule, {-0.055f, 0.375f, 0.0f}, {0.045f, 0.08f, 0.0f}, {-0.055f, 0.50f, 0.0f}, 0.10f},
    {RagdollShape::Capsule, {-0.055f, 0.13f, 0.0f}, {0.04f, 0.085f, 0.0f}, {-0.055f, 0.25f, 0.0f}, 0.045f},
}};

// Size of a part's drawn box (full extents, m) for a body of this height. View helper
inline Vec3 ragdollPartSize(u32 part, f32 height) {
    const RagdollPartLayout& l = kRagdollLayout[part];
    switch (l.shape) {
    case RagdollShape::Box: return l.half * (2.0f * height);
    case RagdollShape::Sphere: return Vec3{l.half.x, l.half.x, l.half.x} * (2.0f * height);
    default: return Vec3{l.half.x, l.half.y + l.half.x, l.half.x} * (2.0f * height);
    }
}

// Joint target rotations (child relative to parent), one per joint. Identity = the rest pose
struct RagdollPose {
    std::array<Quat, kRagdollJoints> joints{};
};

MEALKIT_STRUCT(Ragdoll) {
    MEALKIT_ARRAY(physics::BodyId, kRagdollParts, parts, "Part bodies (RagdollPart order)");
    MEALKIT_ARRAY(physics::JointId, kRagdollJoints, joints, "Joints (joint i drives part i + 1)");
    MEALKIT_FIELD(u32, group, 0, "Collision group (parts ignore each other)");
    MEALKIT_FIELD(f32, height, 0.0f, "Height (m)");
    MEALKIT_FIELD(f32, phase, 0.0f, "Gait phase (radians)");
    MEALKIT_FIELD(u32, limpTicks, 0, "Ticks left knocked down (motors weak, no balance)");
    MEALKIT_ARRAY(Vec3, kRagdollParts, partPositions, "Part positions (for the view)");
    MEALKIT_ARRAY(Quat, kRagdollParts, partRotations, "Part rotations (for the view)");
};

// Pelvis height above the feet in the rest pose
inline f32 ragdollPelvisHeight(const Ragdoll& r) { return kRagdollLayout[0].center.y * r.height; }

// feet: ground point under the body. group: nonzero and unique per ragdoll (e.g. 1 + Pawn slot)
inline Ragdoll createRagdoll(physics::PhysicsWorld& world, Vec3 feet, f32 height, f32 mass, u32 group, f32 maxTorque,
                             f32 motorFrequency) {
    Ragdoll r;
    r.group = group;
    r.height = height;
    for (u32 i = 0; i < kRagdollParts; ++i) {
        const RagdollPartLayout& l = kRagdollLayout[i];
        physics::BodyDesc d;
        d.position = feet + l.center * height;
        d.mass = l.mass * mass;
        d.friction = 0.6f;      // const-ok: grippy feet and hands
        d.restitution = 0.0f;
        d.angularDamping = 0.5f; // const-ok: settles limbs
        d.allowSleep = false;
        d.group = group;
        const Vec3 half = l.half * height;
        if (l.shape == RagdollShape::Box) r.parts[i] = world.createBox(half, d);
        else if (l.shape == RagdollShape::Sphere) r.parts[i] = world.createSphere(half.x, d);
        else r.parts[i] = world.createCapsule(half.y, half.x, d);
        r.partPositions[i] = d.position;
    }
    for (u32 j = 0; j < kRagdollJoints; ++j) {
        const u32 child = j + 1;
        const RagdollPartLayout& l = kRagdollLayout[child];
        physics::JointDesc d;
        d.pivot = feet + l.pivot * height;
        const bool spine = child == static_cast<u32>(RagdollPart::Chest) || child == static_cast<u32>(RagdollPart::Head);
        d.twistAxis = spine ? Vec3{0.0f, 1.0f, 0.0f} : Vec3{0.0f, -1.0f, 0.0f}; // along the bone, away from the parent
        d.planeAxis = {1.0f, 0.0f, 0.0f};
        d.normalCone = spine ? 0.6f : 1.4f; // const-ok: joint limits (radians)
        d.planeCone = spine ? 0.4f : 1.4f;  // const-ok
        d.twistMin = spine ? -0.6f : -0.4f; // const-ok
        d.twistMax = spine ? 0.6f : 0.4f;   // const-ok
        d.motorFrequency = motorFrequency;
        d.maxTorque = maxTorque * l.mass * 4.0f; // const-ok: torque scaled by the part's share (limbs ~0.1..0.3 of the full value)
        r.joints[j] = world.createJoint(r.parts[kRagdollParent[child]], r.parts[child], d);
    }
    return r;
}

inline void destroyRagdoll(physics::PhysicsWorld& world, Ragdoll& r) {
    for (u32 j = kRagdollJoints; j-- > 0;) {
        world.destroyJoint(r.joints[j]);
        r.joints[j] = physics::kInvalidJoint;
    }
    for (u32 i = kRagdollParts; i-- > 0;) {
        world.destroy(r.parts[i]);
        r.parts[i] = physics::kInvalidBody;
    }
}

// Sets the motor targets; torque = full motor torque (scaled per part like createRagdoll), 0 = limp
inline void driveRagdoll(physics::PhysicsWorld& world, const Ragdoll& r, const RagdollPose& pose, f32 torque) {
    for (u32 j = 0; j < kRagdollJoints; ++j) {
        world.setJointTarget(r.joints[j], pose.joints[j]);
        world.setJointTorque(r.joints[j], torque * kRagdollLayout[j + 1].mass * 4.0f); // const-ok: same scale as createRagdoll
    }
}

// Procedural walk: legs and arms swing around the side axis with the phase, knees bend on the forward swing.
// amount 0..1 (0 = standing rest pose), stride in radians
inline RagdollPose ragdollGait(f32 phase, f32 amount, f32 stride) {
    RagdollPose p;
    const f32 swing = detSin(phase) * stride * amount;
    const f32 knee = detCos(phase) * stride * amount;
    const Vec3 side{1.0f, 0.0f, 0.0f};
    auto at = [](RagdollPart part) { return static_cast<u32>(part) - 1; };
    p.joints[at(RagdollPart::ThighL)] = detQuatFromAxisAngle(side, -swing);
    p.joints[at(RagdollPart::ThighR)] = detQuatFromAxisAngle(side, swing);
    p.joints[at(RagdollPart::ShinL)] = detQuatFromAxisAngle(side, knee > 0.0f ? knee : 0.0f);
    p.joints[at(RagdollPart::ShinR)] = detQuatFromAxisAngle(side, knee < 0.0f ? -knee : 0.0f);
    p.joints[at(RagdollPart::UpperArmL)] = detQuatFromAxisAngle(side, swing * 0.7f); // const-ok: arms swing less, opposite
    p.joints[at(RagdollPart::UpperArmR)] = detQuatFromAxisAngle(side, -swing * 0.7f); // const-ok
    p.joints[at(RagdollPart::LowerArmL)] = detQuatFromAxisAngle(side, -0.3f * amount); // const-ok: elbows slightly bent
    p.joints[at(RagdollPart::LowerArmR)] = detQuatFromAxisAngle(side, -0.3f * amount); // const-ok
    return p;
}

// After the physics step: part transforms for the view
inline void senseRagdoll(const physics::PhysicsWorld& world, Ragdoll& r) {
    for (u32 i = 0; i < kRagdollParts; ++i) {
        r.partPositions[i] = world.position(r.parts[i]);
        r.partRotations[i] = world.rotation(r.parts[i]);
    }
}

} // namespace mealkit::sim
