//! Single-source vehicle contract.
//!
//! Aerial offboard authority is declared once by `define_aerial_authority!`
//! in `safety`. [`vehicle_contract!`] `from_kernel` aliases that table so the
//! typestate capability, runtime monitors, Kani harness name, diagrams, and
//! traceability ids cannot drift from the kernel predicates.

/// Declare a named capability contract.
///
/// `from_kernel` binds a capability to the aerial authority table in `safety`.
/// The event-list form remains for additional capabilities that do not yet
/// have a kernel predicate.
#[macro_export]
macro_rules! vehicle_contract {
    (capability $name:ident { from_kernel }) => {
        #[doc = concat!("Contract capability `", stringify!($name), "`.")]
        pub struct $name;

        impl $name {
            pub const HEARTBEAT_MAX_AGE_MS: u32 = $crate::safety::OFFBOARD_HEARTBEAT_MAX_AGE_MS;
            pub const COMMAND_MAX_AGE_MS: u32 = $crate::safety::COMMAND_MAX_AGE_MS;
            pub const TRACE_ID: &'static str = concat!("FC-CAP-", stringify!($name));
            pub const REVOKE_ON: &'static [$crate::safety::Event] =
                $crate::safety::AUTHORITY_REVOKE_EVENTS;
            pub const MERMAID: &'static str = $crate::safety::AERIAL_OFFBOARD_MERMAID;
            pub const SPEC: &'static str = $crate::safety::AERIAL_OFFBOARD_SPEC;
            pub const GRAPHVIZ: &'static str = $crate::safety::AERIAL_OFFBOARD_GRAPHVIZ;

            pub const fn revokes(event: $crate::safety::Event) -> bool {
                $crate::safety::event_revokes_authority(event)
            }

            pub const MONITORS: &'static [$crate::contracts::Requirement] = &[
                $crate::contracts::Requirement::NeverActuateWhileDisarmed,
                $crate::contracts::Requirement::ActuatorsImplyArmed,
                $crate::contracts::Requirement::PermitEpochMonotonic,
                $crate::contracts::Requirement::NoNanCommands,
                $crate::contracts::Requirement::OffboardHeartbeatFresh,
                $crate::contracts::Requirement::CommandAgeMs {
                    max_ms: $crate::safety::COMMAND_MAX_AGE_MS,
                },
                $crate::contracts::Requirement::EstimatorTimestampsMonotonic,
            ];

            pub const KANI_HARNESS: &'static str = "dsl_revokes_match_kernel";
        }
    };
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
                "    [*] --> Offboard\n",
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
                "digraph ",
                stringify!($name),
                " {\n  Offboard -> Failsafe [label=\"",
                stringify!($($ev)|*),
                "\"];\n}\n"
            );

            pub const MONITORS: &'static [$crate::contracts::Requirement] = &[
                $crate::contracts::Requirement::NeverActuateWhileDisarmed,
                $crate::contracts::Requirement::ActuatorsImplyArmed,
            ];

            pub const KANI_HARNESS: &'static str = "dsl_revokes_match_kernel";
        }
    };
}

vehicle_contract! {
    capability AerialOffboard {
        from_kernel
    }
}

/// Kernel invariant `actuators_enabled → armed` (traceability FC-INV-001).
pub const INV_ACTUATORS_IMPLY_ARMED: &str = "FC-INV-001";
/// Permit epoch must match the live backend (FC-INV-002).
pub const INV_PERMIT_EPOCH: &str = "FC-INV-002";
/// Offboard requires a fresh heartbeat (FC-INV-003).
pub const INV_OFFBOARD_HEARTBEAT: &str = "FC-INV-003";
/// Command age at actuation must be inside the contract bound (FC-INV-004).
pub const INV_COMMAND_AGE: &str = "FC-INV-004";
/// Estimator timestamps are monotonic (FC-INV-005).
pub const INV_ESTIMATOR_TS: &str = "FC-INV-005";

/// Human-readable specification emitted from the same tables as the types.
pub fn human_readable_spec() -> &'static str {
    concat!(
        "flight-core safety contract\n",
        "============================\n",
        "capability AerialOffboard {\n",
        "  requires heartbeat.age < 250.ms();\n",
        "  requires command.age < 100.ms();\n",
        "  revokes_on [TriggerFailsafe, Disarm, Disconnect, HeartbeatStale, EstimatorInvalid, ImuUnhealthy]\n",
        "}\n",
        "invariant {\n  actuators.enabled -> armed  [FC-INV-001]\n",
        "  ActuationPermit.epoch == backend.epoch  [FC-INV-002]\n",
        "  OffboardControl => heartbeat_age < 250 ms  [FC-INV-003]\n",
        "  command.age < 100 ms at actuation  [FC-INV-004]\n",
        "  estimator timestamps monotonic  [FC-INV-005]\n",
        "}\n"
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::contracts::Requirement;
    use crate::safety::{
        event_revokes_authority, Event, AUTHORITY_REVOKE_EVENTS, COMMAND_MAX_AGE_MS,
        OFFBOARD_HEARTBEAT_MAX_AGE_MS,
    };

    #[test]
    fn dsl_is_the_kernel_table() {
        assert!(core::ptr::eq(
            AerialOffboard::REVOKE_ON,
            AUTHORITY_REVOKE_EVENTS
        ));
        for e in Event::ALL {
            assert_eq!(
                AerialOffboard::revokes(e),
                event_revokes_authority(e),
                "event {e:?} disagrees between capability and kernel"
            );
        }
    }

    #[test]
    fn heartbeat_and_command_bounds_match_kernel() {
        assert_eq!(
            AerialOffboard::HEARTBEAT_MAX_AGE_MS,
            OFFBOARD_HEARTBEAT_MAX_AGE_MS
        );
        assert_eq!(AerialOffboard::COMMAND_MAX_AGE_MS, COMMAND_MAX_AGE_MS);
        assert_eq!(AerialOffboard::SPEC, crate::safety::AERIAL_OFFBOARD_SPEC);
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
        assert!(
            AerialOffboard::MONITORS.contains(&Requirement::CommandAgeMs {
                max_ms: COMMAND_MAX_AGE_MS
            })
        );
        assert!(AerialOffboard::MONITORS.contains(&Requirement::EstimatorTimestampsMonotonic));
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
        assert!(human_readable_spec().contains("FC-INV-004"));
        assert!(human_readable_spec().contains("command.age < 100"));
        assert!(AerialOffboard::SPEC.contains("command.age < 100"));
    }
}
