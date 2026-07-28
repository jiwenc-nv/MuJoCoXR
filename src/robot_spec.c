#include "robot_spec.h"

#include <stdio.h>
#include <string.h>

// ---------------------------------------------------------------------------
// The robot table.
//
// ADDING A ROBOT — and the list of files that must NOT change, which is the
// design's actual claim — is docs/adding-a-robot.md. It is not restated here:
// this comment used to carry a second copy that had already drifted from the
// doc's before either was read (it omitted scripts/fetch-menagerie.sh, without
// which the scene silently never stages).
// ---------------------------------------------------------------------------
static const MxrRobot kRobots[] = {
    {
        .tcp_body = "hand",
        .joint = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
                  "joint7"},
        .gripper_act = "actuator8",
        // Grasp midpoint in the hand frame, 103 mm down the +z tool axis.
        .tcp_offset = {0, 0, 0.103},
        // 1.0 because the Panda is full-rank for a 6D task: the natural
        // metric is achievable, so any weight is a distortion. Measured
        // monotone — the fingertip error is worst at every value below 1.0.
        .w_rot = 1.0,
        .lambda = 0.05,
        // 7 joints against a 6D task leaves a genuine 1-dimensional
        // nullspace, so this is a real projector here and the bias costs the
        // task nothing. Contrast the SO101 row.
        .ns_gain = 0.1,
        .clutch_scale = 1.0,
        // panda.xml:275 says in its own comment that this is a per-model
        // remap. 0 = closed, 255 = open.
        .gripper_closed = 0.0,
        .gripper_open = 255.0,
    },
    {
        .tcp_body = "gripper",
        // Five. The SO101 is one rotational DOF short of the 6D task, which
        // is the single fact that drives w_rot, ns_gain and clutch_scale
        // below away from the Franka's values.
        .joint = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex",
                  "wrist_roll"},
        .gripper_act = "gripper",
        // so101.xml's own `gripperframe` site, verbatim: (0.012, -0.000218,
        // -0.098127) in the `gripper` body frame. Verified to land on the
        // site's world position to the printed digit at home,
        // (0.2735, 0.0118, 0.0899). Note the tool axis is -z here and +z on
        // the Franka; see MxrRobot::tcp_body.
        .tcp_offset = {0.012, -0.000218, -0.098127},
        // 0.05: the measured knee, three independent sweeps agreeing, over
        // the 360-frame bench/teleop_replay script.
        //
        //   w_rot | pos med / p90 / max (mm) | ori med / p90 / max (deg)
        //   1.00  |  55.8 /  91.6 / 95.1     |  1.0 /  1.4 /  3.7
        //   0.20  |  19.2 /  40.3 / 42.2     |  7.7 / 13.9 / 14.5
        //   0.10  |   6.9 /  14.0 / 14.7     | 10.7 / 20.0 / 20.9
        //   0.05  |   3.3 /   4.5 /  5.5     | 11.6 / 22.2 / 23.2
        //   0.02  |   2.1 /   3.9 /  5.6     | 11.9 / 23.0 / 24.0
        //   0.00  |   1.7 /   3.3 /  5.6     | 34.2 / 51.0 / 52.1
        //
        // 0.10 -> 0.05 buys half the position error for 0.9 deg of median
        // orientation; 0.05 -> 0 costs 22.6 deg to buy 1.6 mm. Both endpoints
        // are bad, which is why this is a tuned scalar and not a mode flag.
        //
        // A SECOND, DIFFERENT TRAJECTORY, recorded because the two numbers
        // must not be read as one refuting the other: over a sweep of RANDOM
        // REACHABLE targets — a workspace the shipping script provably never
        // enters (0/360 clamped frames, 0 contacts) — w_rot = 0.05 has a
        // ~155 mm max against ~40 mm at 0.10. That tail is the workspace-edge
        // fold: near the reach limit the arm cannot satisfy position either,
        // and a low w_rot stops trading orientation away to try. Nothing in
        // this file fixes it, the domain-correct fix is a manipulability
        // clamp on the TARGET, and if a researcher reports it as an IK bug
        // this is the paragraph to read them. THIS IS THE ONE LITERAL TO
        // RAISE if the first on-device session works near full extension.
        .w_rot = 0.05,
        // Shared with the Franka deliberately. A settle-only argument for
        // 0.03 was withdrawn after measurement: in motion lambda is flat
        // (0.02 -> 0.10 moves the median 3.2 -> 3.3 mm, non-monotone), and at
        // w_rot = 0.05 the smallest singular value is 0.0207, so
        // lambda^2/sigma^2 = 5.8 and the damping is doing real work holding
        // the near-singular direction together. Lowering it trades a
        // sub-millimetre interior number against the tail above.
        .lambda = 0.05,
        // ZERO, and this is a correctness choice rather than a tuning one.
        //
        // The SO101's J is 6x5 with singular values 1.769 / 1.321 / 0.556 /
        // 0.0953 / 0.0807 — full COLUMN rank, so dim N(J) = 0. THERE IS NO
        // NULLSPACE ON THIS ARM, and `ns_gain` is named for an operation that
        // does not exist here. What ik_dls.c actually computes is the DAMPED
        // projector I - J'(JJ' + lambda^2 I)^-1 J, which is not a projector
        // and leaks as lambda^2/sigma^2: 0.55 % at w_rot = 1.0 but 27.6 % at
        // the 0.05 shipped above — 119x more, because w_rot scales the
        // rotation rows and shrinks sigma with them.
        //
        // That leak lands on the task command. Measured at w_rot = 0.05, on a
        // pitch axis the arm hits EXACTLY: ns_gain = 0.1 produces 1.13 mm and
        // 5.43 deg of uncommanded motion, ns_gain = 0.3 produces 1.85 mm and
        // 9.13 deg, against 0.00 mm / 0.00 deg at zero. A commanded pure
        // translation must produce a pure translation — that is the property
        // a teleoperator's hand-eye loop is closed around, and 7 deg of
        // spurious tool roll is the most disorienting thing you can hand
        // them. It went unnoticed for two rounds because the metric everyone
        // was watching was a POSITION median, which cannot see a rotation.
        .ns_gain = 0.0,
        // 0.5, the highest-value number measured on this arm. At 1.0 the
        // 360-frame script pins a joint against its stop on 69 frames (19 %)
        // with a 22.8 mm worst error; at 0.5, 0 frames and 14.0 mm. The
        // Franka's workspace swallows a 1:1 hand mapping and this arm's does
        // not, so the control-display ratio is where the reach difference is
        // absorbed — not in the rate limits, which are shared (see
        // src/teleop.cc) and were measured to make lag WORSE when tightened.
        .clutch_scale = 0.5,
        // /code/Lab's SO101_GRIPPER_CLOSE / _OPEN, the identical affine map.
        // 1.745 rad is inside so101.xml's ctrlrange of (-0.17453, 1.74533),
        // which src/teleop.cc re-checks at init rather than assuming.
        //
        // gripper_closed = 0.0 DOES NOT CLOSE THE JAWS, and that is imported
        // from Lab along with the constant. Measured aperture (centroid of
        // the three fixed_jaw_sph_tip* geoms to the centroid of the three
        // moving_jaw_sph_tip*, at the home posture): 1.745 -> 129.9 mm,
        // 0.0 -> 16.3 mm, -0.17453 -> 4.4 mm. So a fully squeezed trigger
        // stops 16 mm apart: it cannot pinch anything thinner than that, it
        // applies full rated torque to anything thicker, and exactly zero at
        // 16 mm. -0.17453 is INSIDE ctrlrange — the travel is there and this
        // map declines to use it.
        //
        // Kept at Lab's value anyway, because Lab's 0.0 is right FOR LAB: a
        // binary open/close action on ~40 mm cubes, where the last 16 mm is
        // wasted stroke. A continuous teleop trigger is a different
        // instrument and 0.0 is probably wrong for it. Not changed here
        // because it moves this scene's golden trace and the right value is a
        // grasp question, not a sim one. TRIGGER: the first on-device session
        // that tries to pick up anything thin -> set this to -0.174533 and
        // re-record baselines/teleop-so101-host-x86_64.txt.
        .gripper_closed = 0.0,
        .gripper_open = 1.745,
    },
};

// ---------------------------------------------------------------------------
// The scene table. Menu order. See MxrScene::id — `id` is the only name.
// ---------------------------------------------------------------------------
static const MxrScene kScenes[] = {
    {.id = "franka", .label = "Franka Emika Panda"},
    {.id = "so101", .label = "SO-101"},
};

enum {
  kNumRobots = (int)(sizeof(kRobots)/sizeof(kRobots[0])),
  kNumScenes = (int)(sizeof(kScenes)/sizeof(kScenes[0])),
};

int mxr_robot_narm(const MxrRobot* r) {
  int n = 0;
  // Bounded by the capacity as well as by the NULL sentinel: the Franka fills
  // all MXR_MAX_ARM slots, so a full row has no terminator to find. Shorter
  // rows are zero-filled by the designated initialisers above, which is what
  // makes "the leading run of non-NULL entries" the count.
  while (n < MXR_MAX_ARM && r->joint[n]) {
    ++n;
  }
  return n;
}

const MxrRobot* mxr_robot_probe(const mjModel* m, const char** why) {
  static char msg[320];
  char matched[128];
  const MxrRobot* found = NULL;
  int nmatch = 0;
  int n = 0;
  matched[0] = '\0';
  // Written unconditionally so a caller never has to seed it, and cleared
  // first so success is observably NULL rather than whatever was left over.
  if (why) {
    *why = NULL;
  }
  // Probe EVERY row rather than returning the first hit, so an ambiguous
  // table is a loud failure at load instead of a robot that silently drives
  // with another robot's gains.
  for (int i = 0; i < kNumRobots; ++i) {
    if (mj_name2id(m, mjOBJ_BODY, kRobots[i].tcp_body) >= 0) {
      ++nmatch;
      found = &kRobots[i];
      if (n >= 0 && n < (int)sizeof(matched)) {
        int w = snprintf(matched + n, sizeof(matched) - n, "%s'%s'",
                         nmatch > 1 ? ", " : "", kRobots[i].tcp_body);
        n = (w < 0) ? n : n + w;
      }
    }
  }
  if (nmatch == 1) {
    return found;
  }
  // Both failures name the discriminator, because the fix is always in the
  // tcp_body column: no match means a model this table does not describe, and
  // more than one means two rows that no longer separate.
  if (nmatch == 0) {
    snprintf(msg, sizeof(msg),
             "no robot in src/robot_spec.c matches this model — none of its "
             "%d tcp_body names resolve against it. Either this scene's robot "
             "has no table row, or its wrapper XML did not include the robot.",
             kNumRobots);
  } else {
    snprintf(msg, sizeof(msg),
             "%d of %d robots in src/robot_spec.c match this model (tcp_body "
             "%s). The table no longer discriminates, so the gains would be a "
             "guess; give the rows distinct tcp_body names.",
             nmatch, kNumRobots, matched);
  }
  if (why) {
    *why = msg;
  }
  return NULL;
}

int mxr_scene_count(void) { return kNumScenes; }

const MxrScene* mxr_scene_at(int i) {
  return (i >= 0 && i < kNumScenes) ? &kScenes[i] : NULL;
}

const MxrScene* mxr_scene_by_id(const char* id) {
  if (!id) {
    return NULL;
  }
  for (int i = 0; i < kNumScenes; ++i) {
    if (!strcmp(kScenes[i].id, id)) {
      return &kScenes[i];
    }
  }
  return NULL;
}
