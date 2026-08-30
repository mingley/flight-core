//! Single-source vehicle contract.
//!
//! The [`vehicle_contract`] macro is the human-facing specification. It
//! expands to capability tables that the typestate API, runtime kernel
//! revoke list, Kani harnesses, monitors, diagrams, and traceability ids
//! must match. A unit test fails if the kernel's
//! [`crate::safety::event_revokes_authority`] disagrees with the table.

/// Declare a named capability contract.
///
/// Generates a zero-sized type with heartbeat bound, revoke list,
/// traceability id, mermaid, and a human-readable specification string.
#[macro_export]
macro_rules! vehicle_contract {
    (
        capability $name:ident {
            heartbeat_age_ms: $hb:expr,
            revokes_on: [$($ev:ident),* $(,)?]
        }
    ) => {
        #[doc = concat!("Contract capability `", stringify!($name), "`.")]
        pub struct $name;

        impl $name {
            pub const HEARTBEAT_MAX_AGE_MS: u32 = $hb;
            pub const TRACE_ID: &'static str = concat!("FC-CAP-", stringify!($name));
            pub const REVOKE_ON: &'static [$crate::safety::Event] = &[
                $($crate::safety::Event::$ev),*
            ];
            pub const MERMAID: &'static str = concat!(
                "stateDiagram-v2\n",
                "    [*] --> Disarmed\n",
                "    Disarmed --> PreflightReady: verify_preflight\n",
                "    PreflightReady --> Armed: arm\n",
                "    Armed --> Offboard: acquire_offboard_control\n",
                "    Offboard --> Failsafe: ",
                stringify!($($ev)|*),
                "\n"
            );
            pub const SPEC: &'static str = concat!(
                "capability ",
                stringify!($name),
                " {\n  requires heartbeat.age < ",
                stringify!($hb),
                ".ms();\n  revokes_on [",
                stringify!($($ev),*),
                "]\n}\n"
            );

            pub const fn revokes(event: $crate::safety::Event) -> bool {
                match event {
                    $($crate::safety::Event::$ev => true,)*
                    _ => false,
                }
            }

            pub const GRAPHVIZ: &'static str = concat!(
                "digraph AerialOffboard {\n",
                "  rankdir=LR;\n",
                "  Disarmed -> PreflightReady [label=verify_preflight];\n",
                "  PreflightReady -> Armed [label=arm];\n",
                "  Armed -> Offboard [label=acquire_offboard_control];\n",
                "  Offboard -> Failsafe [label=\"",
                stringify!($($ev)|*),
                "\"];\n",
                "}\n"
            );

            pub const MONITORS: &'static [$crate::contracts::Requirement] = &[
                $crate::contracts::Requirement::NeverActuateWhileDisarmed,
                $crate::contracts::Requirement::ActuatorsImplyArmed,
                $crate::contracts::Requirement::PermitEpochMonotonic,
                $crate::contracts::Requirement::NoNanCommands,
                $crate::contracts::Requirement::OffboardHeartbeatFresh,
            ];

            pub const KANI_HARNESS: &'static str = "dsl_revokes_match_kernel";
        }
    };
}

vehicle_contract! {
    capability AerialOffboard {
        heartbeat_age_ms: 250,
        revokes_on: [
            TriggerFailsafe,
            Disarm,
            Disconnect,
            HeartbeatStale,
            EstimatorInvalid,
            ImuUnhealthy
        ]
    }
}

/// Kernel invariant `actuators_enabled → armed` (traceability FC-INV-001).
pub const INV_ACTUATORS_IMPLY_ARMED: &str = "FC-INV-001";
/// Permit epoch must match the live backend (FC-INV-002).
pub const INV_PERMIT_EPOCH: &str = "FC-INV-002";
/// Offboard requires a fresh heartbeat (FC-INV-003).
pub const INV_OFFBOARD_HEARTBEAT: &str = "FC-INV-003";

/// Human-readable specification emitted from the same tables as the types.
pub fn human_readable_spec() -> &'static str {
    concat!(
        "flight-core safety contract\n",
        "============================\n",
        "capability AerialOffboard {\n",
        "  requires heartbeat.age < 250.ms();\n",
        "  revokes_on [TriggerFailsafe, Disarm, Disconnect, HeartbeatStale, EstimatorInvalid, ImuUnhealthy]\n",
        "}\n",
        "invariant {\n  actuators.enabled -> armed  [FC-INV-001]\n",
        "  ActuationPermit.epoch == backend.epoch  [FC-INV-002]\n",
        "  OffboardControl => heartbeat_age < 250 ms  [FC-INV-003]\n",
        "}\n"
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::contracts::Requirement;
    use crate::safety::{event_revokes_authority, Event, OFFBOARD_HEARTBEAT_MAX_AGE_MS};

    #[test]
    fn dsl_matches_kernel_revoke_function() {
        for e in Event::ALL {
            assert_eq!(
                AerialOffboard::revokes(e),
                event_revokes_authority(e),
                "event {e:?} disagrees between DSL and kernel"
            );
        }
    }

    #[test]
    fn heartbeat_bound_matches_kernel() {
        assert_eq!(
            AerialOffboard::HEARTBEAT_MAX_AGE_MS,
            OFFBOARD_HEARTBEAT_MAX_AGE_MS
        );
    }

    #[test]
    fn revoke_table_is_the_named_set() {
        assert!(AerialOffboard::REVOKE_ON.contains(&Event::TriggerFailsafe));
        assert!(AerialOffboard::REVOKE_ON.contains(&Event::Disarm));
        assert!(!AerialOffboard::REVOKE_ON.contains(&Event::MissionCommand));
        assert!(AerialOffboard::TRACE_ID.starts_with("FC-CAP-"));
        assert!(AerialOffboard::MERMAID.contains("Offboard"));
        assert!(AerialOffboard::GRAPHVIZ.contains("digraph"));
        assert_eq!(AerialOffboard::KANI_HARNESS, "dsl_revokes_match_kernel");
        assert!(AerialOffboard::MONITORS.contains(&Requirement::OffboardHeartbeatFresh));
        let generated = include_str!("../../../../docs/generated/aerial-offboard.mmd");
        fn tokens(s: &str) -> Vec<&str> {
            s.split_whitespace().collect()
        }
        assert_eq!(
            tokens(AerialOffboard::MERMAID),
            tokens(generated),
            "docs/generated/aerial-offboard.mmd must match AerialOffboard::MERMAID"
        );
        assert!(human_readable_spec().contains("FC-INV-001"));
    }
}
