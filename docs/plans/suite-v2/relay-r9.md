# R9 mixed-axis relay hinges — 2026-10-25

Only the three L3 environments change. R8 L2, L1 and the other families retain
their exact source inputs. R8 L3 models and previews remain archived.

| Arms | Payloads with in-plane hinge | Payloads with out-of-plane hinge |
| --- | --- | --- |
| 8 | 01 | 02 |
| 12 | 01 | 02,03 |
| 16 | 01,02 | 03,04 |

In-plane hinges rotate about beam-local Z; the visible pin and spring coil are
vertical in the tabletop neutral pose. Out-of-plane hinges rotate about local
X; their bearing and coil are horizontal in that pose. Neither axis stays fixed
to the world when a robot turns the whole beam. Each manufactured hinge has
one passive axis, not a runtime-selectable mode.

Both modes retain the R8 spring stiffness18 Nm/rad, damping0.55 Nms/rad,
zero-angle rest and nominal +/-25-degree limits. In-plane closure therefore
changes the nominal135-degree angle toward110 degrees, not an acute angle.
The weight, short rigid pendulum, closed-eye contact suspension, layout,
actuators and2 ms timestep are unchanged. The visible pin orientation, compiled
joint axis and per-payload public mechanism description agree.

All three new initial scenes pass the static geometry checks. Targeted tests
check the exact1+1,1+2,2+2 assignments, motion direction, bearing-axis agreement,
replay registry, evaluation and recording;33 regressions pass.

`docs/generated/frontier-v2/relay-r9/checks.json` also contains five-second
passive suspended fixtures for both modes at2 and1 ms, starting with0.15 rad
of hinge displacement. The independent weight pose is transformed consistently
with the displaced distal half; no robot or live world is manipulated.
Neither mode detaches or produces numerical warnings. The disturbed
out-of-plane fixture has transient ring penetration up to approximately1.22 mm
at2 ms (0.92 mm at1 ms). Contact fidelity therefore still needs dynamic review;
this is NOT a full physical acceptance, material calibration or task witness.
Task readiness remains`environment_preview_only`; no Oracle or policy is run.

The earlier R8 timing comparison remains scoped to R8; no new task speedup is
claimed. Factory composition still uses R7 source assets and is pending an
explicit rebuild. Stable body/joint identities and generic recording remain
available for later offline transfer.

Current snapshot:`configs/suites/frontier-v2-r9.json`.
Gallery:`docs/generated/frontier-v2/relay-r9/index.html`.
Direction close-up:`relay-r9/hinge-directions.png`.
Source:`rigid_relay.configure_hinge`, invoked once per fresh L3 R9 clone by
`v2.distribute_payloads`. No new scheduler, control tool or task planner.

## 5 ms candidate rejected

Read-only comparison:`mechanism-checks/rigid-timestep-comparison.json` under
`docs/generated/frontier-v2`. Three passive suspension fixtures,5 simulated
seconds each,3 repeats at1/2/5 ms. All remain linked without numerical warnings,
but ring penetration at2 versus5 ms increases from0.454 to3.386 mm (L2),
0.206 to1.185 mm (in-plane L3), and1.216 to2.893 mm (out-of-plane L3).
Ring stock diameter is5 mm. The fixture computation is approximately2.4x faster
at5 ms, but the contact change is substantial; no global switch is made.
MuJoCo refsafety is enabled: the declared4 ms contact time constant is internally
bounded by2*timestep, hence at least10 ms at5 ms integration. Reducing the step
count therefore also changes effective contact softness in this comparison.
All36 formal scenes retain2 ms. This does not certify2 ms dynamic contact
accuracy or establish5 ms behaviour for every other family. No Oracle is run.
