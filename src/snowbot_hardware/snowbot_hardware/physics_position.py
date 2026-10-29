"""Snowbot position IO backed by acknowledged physics feedback."""
import math
from .protocol import SnowbotIO


class PhysicsPositionIO(SnowbotIO):
    """Confirm an explicit device hold at its acknowledged physical position."""
    def __init__(self, *args, physics, **kwargs):
        super().__init__(*args, **kwargs)
        self.physics = physics
        self.hold_sequences = {}
        self.hold_ack = None

    def capture(self, groups):
        super().capture(groups)
        self.hold_sequences.clear()
        self.hold_ack = None

    def stop(self):
        if not self.targets:
            return super().stop()  # Preserve an outstanding request and deadline.
        self.reference.clear()
        self.stop_requested_at = self.now()
        self.stop_targets = {}
        self.hold_sequences.clear()
        self.hold_ack = None
        groups = self._groups(self.targets)
        try:
            self.require_feedback()
        except RuntimeError:
            self.stop_failures = tuple(sorted(groups))
            self.targets.clear()
            return
        failures = set(self.stop_failures)
        for group in sorted(groups):
            # Reserve the entire footprint even if a send partly fails.
            self.stop_targets.update({n:self.last_feedback.positions[n]
                                      for n in self.profile.groups[group]})
            try:
                self.hold_sequences[group] = self.physics.send(group, operation='hold')
            except RuntimeError:
                failures.add(group)
        self.stop_failures = tuple(sorted(failures))
        self.targets.clear()

    def confirmed_stop_targets(self):
        """Wait for all post-step ACKs and matching fresh joint feedback.

        Acknowledge each request once; never chase drifting measured positions
        or accept the timeout fallback hold as evidence of this explicit hold.
        """
        h = self.physics.health
        at = h.get('stamp')
        if (self.stop_failures or not self.hold_sequences or self.physics.failed or
                not h.get('ready') or h.get('simulator_id') != self.physics.simulator_id or
                type(at) not in (int,float) or not math.isfinite(at) or
                at <= self.stop_requested_at or not 0 <= self.now()-at <= .2):
            return None
        sequences, holds = h.get('applied_sequences'), h.get('hold_positions')
        if not isinstance(sequences,dict) or not isinstance(holds,dict):
            return None
        if any(sequences.get(g) != seq for g,seq in self.hold_sequences.items()):
            return None
        acknowledgments=h.get('hold_acknowledgments')
        if not isinstance(acknowledgments,dict):
            return None
        targets={}
        ack_stamps=[]
        for group,sequence in self.hold_sequences.items():
            ack=acknowledgments.get(group)
            if not isinstance(ack,dict):return None
            positions,ack_at=ack.get('positions'),ack.get('stamp')
            if (ack.get('sequence')!=sequence or not isinstance(positions,dict) or
                    set(positions)!=set(self.profile.groups[group]) or
                    type(ack_at) not in (int,float) or not math.isfinite(ack_at) or
                    not self.stop_requested_at < ack_at <= at):
                return None
            targets.update(positions)
            ack_stamps.append(ack_at)
        if set(targets)!=set(self.stop_targets):return None
        tolerance=self.profile.feedback_tolerances['joint_position']
        # Measured braking points can slightly cross a nominal limit. Preserve
        # the actual ACK; the command gate still enforces the nominal limits.
        if any(type(q) not in (float,int) or not math.isfinite(q) or
               not self.profile.limits[n][0]-tolerance <= q <= self.profile.limits[n][1]+tolerance
               for n,q in targets.items()):
            return None
        if self.hold_ack is None:
            self.hold_ack = (max(ack_stamps), targets)
        ack_stamp, acknowledged = self.hold_ack
        if targets != acknowledged or self.last_source_stamp+1e-6 < ack_stamp:
            return None
        return dict(acknowledged)
