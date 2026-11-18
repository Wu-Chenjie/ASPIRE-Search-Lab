"""Memory semantics for already-associated traces, not a trace detector.

The caller supplies a stable trace ID and trustworthy geometry. No person ID
or person count is inferred. Observing a trace again does not refresh its
physical formation time. Time values use one caller-chosen clock domain.
"""

from dataclasses import dataclass
from enum import Enum
from math import hypot, isfinite
from typing import ClassVar, Dict, Optional, Tuple

if __package__:
    from .trace_contracts import SurfaceEstimate, _finite_number, nonempty, nonnegative_finite, unit_interval
else:
    from trace_contracts import SurfaceEstimate, _finite_number, nonempty, nonnegative_finite, unit_interval


class TraceType(str, Enum):
    FOOTPRINT = "footprint"
    DISTURBED_REGION = "disturbed_region"
    FALLEN_MATERIAL = "fallen_material"


class DirectionState(str, Enum):
    UNKNOWN = "unknown"
    AXIAL = "axial"
    DIRECTED = "directed"


@dataclass(frozen=True)
class TraceDirection:
    state: DirectionState = DirectionState.UNKNOWN
    axis_xy: Optional[Tuple[float, float]] = None
    confidence: float = 0.0

    def __post_init__(self) -> None:
        if not isinstance(self.state, DirectionState):
            raise ValueError("state must be DirectionState")
        unit_interval(self.confidence, "direction confidence")
        if self.state == DirectionState.UNKNOWN:
            if self.axis_xy is not None or self.confidence != 0:
                raise ValueError("unknown direction must have no axis and zero confidence")
            return
        if not isinstance(self.axis_xy, (tuple, list)) or len(self.axis_xy) != 2 or any(
            not _finite_number(v) for v in self.axis_xy
        ):
            raise ValueError("a known direction requires a finite two-dimensional axis")
        length = hypot(*self.axis_xy)
        if length == 0 or not isfinite(length):
            raise ValueError("direction axis must be nonzero and finite")
        x, y = (v / length for v in self.axis_xy)
        if self.state == DirectionState.AXIAL and (x < 0 or (x == 0 and y < 0)):
            x, y = -x, -y
        object.__setattr__(self, "axis_xy", (x, y))

    def possible_directions(self) -> Tuple[Tuple[float, float], ...]:
        if self.state == DirectionState.UNKNOWN:
            return ()
        if self.state == DirectionState.AXIAL:
            x, y = self.axis_xy
            return ((x, y), (-x, -y))
        return (self.axis_xy,)


@dataclass(frozen=True)
class FormationInterval:
    earliest: float
    latest: float
    evidence_source: str

    def __post_init__(self) -> None:
        nonnegative_finite(self.earliest, "earliest")
        nonnegative_finite(self.latest, "latest")
        nonempty(self.evidence_source, "evidence_source")
        if self.earliest > self.latest:
            raise ValueError("formation interval is reversed")


@dataclass(frozen=True)
class TraceObservation:
    observation_id: str
    trace_id: str
    observed_at: float
    trace_type: TraceType
    semantic_confidence: float
    direction: TraceDirection = TraceDirection()
    support_position: Optional[SurfaceEstimate] = None
    formation_time: Optional[FormationInterval] = None

    def __post_init__(self) -> None:
        nonempty(self.observation_id, "observation_id")
        nonempty(self.trace_id, "trace_id")
        nonnegative_finite(self.observed_at, "observed_at")
        unit_interval(self.semantic_confidence, "semantic_confidence")
        if not isinstance(self.trace_type, TraceType) or not isinstance(self.direction, TraceDirection):
            raise ValueError("trace type and direction must use the declared contracts")
        if self.support_position is not None:
            if not isinstance(self.support_position, SurfaceEstimate):
                raise ValueError("support_position must be SurfaceEstimate")
            if self.support_position.stamp != self.observed_at:
                raise ValueError("current trace geometry must belong to this observation")
        if self.formation_time is not None:
            if not isinstance(self.formation_time, FormationInterval):
                raise ValueError("formation_time must be FormationInterval")
            if self.formation_time.latest > self.observed_at:
                raise ValueError("a trace cannot form after it was observed")


@dataclass(frozen=True)
class TraceRecord:
    entity_kind: ClassVar[str] = "search_trace"
    contributes_to_person_count: ClassVar[bool] = False
    trace_id: str
    trace_type: TraceType
    first_observed_at: float
    last_observed_at: float
    observation_count: int
    semantic_confidence: float
    direction: TraceDirection
    current_support_position: Optional[SurfaceEstimate]
    last_reliable_support: Optional[SurfaceEstimate]
    formation_time: Optional[FormationInterval]

    def observation_age_s(self, now: float) -> float:
        nonnegative_finite(now, "now")
        if now < self.last_observed_at:
            raise ValueError("now precedes the last observation")
        return now - self.last_observed_at

    def physical_age_bounds_s(self, now: float) -> Optional[Tuple[float, float]]:
        self.observation_age_s(now)
        if self.formation_time is None:
            return None
        return (now - self.formation_time.latest, now - self.formation_time.earliest)


class TraceMemory:
    """Ordered, single-threaded mission memory with explicit duplicate checks.

    This is not spatial data association. Unknown formation times stay unknown;
    new physical-age evidence must overlap earlier evidence for the same trace.
    """

    def __init__(self) -> None:
        self._records: Dict[str, TraceRecord] = {}
        self._observations: Dict[str, TraceObservation] = {}

    def get(self, trace_id: str) -> Optional[TraceRecord]:
        return self._records.get(trace_id)

    def observe(self, observation: TraceObservation) -> TraceRecord:
        if not isinstance(observation, TraceObservation):
            raise ValueError("observation must be TraceObservation")
        previous_observation = self._observations.get(observation.observation_id)
        if previous_observation is not None:
            if previous_observation != observation:
                raise ValueError("an observation ID cannot be reused with different content")
            return self._records[observation.trace_id]
        previous = self._records.get(observation.trace_id)
        formation = observation.formation_time
        if previous is not None:
            if observation.trace_type != previous.trace_type:
                raise ValueError("reclassification requires an explicit new trace record")
            if observation.observed_at < previous.last_observed_at:
                raise ValueError("out-of-order observations need an explicit replay/fusion policy")
            prior = previous.formation_time
            if formation is None:
                formation = prior
            else:
                # Seeing the same trace earlier is an upper bound on formation.
                start, end = formation.earliest, formation.latest
                source = formation.evidence_source
                if start > previous.first_observed_at:
                    raise ValueError("formation evidence postdates the first sighting of this trace")
                if end > previous.first_observed_at:
                    end = previous.first_observed_at
                    source += " | first sighting"
                if prior is not None:
                    start = max(start, prior.earliest)
                    end = min(end, prior.latest)
                    if start > end:
                        raise ValueError("conflicting formation evidence; do not refresh the trace age")
                    source = " | ".join(sorted({source, prior.evidence_source}))
                if prior is not None or end != formation.latest:
                    formation = FormationInterval(start, end, source)
        history = observation.support_position
        if history is None and previous is not None:
            history = previous.last_reliable_support
        record = TraceRecord(
            trace_id=observation.trace_id,
            trace_type=observation.trace_type,
            first_observed_at=previous.first_observed_at if previous else observation.observed_at,
            last_observed_at=observation.observed_at,
            observation_count=previous.observation_count + 1 if previous else 1,
            # Correlated frames are not accumulated as independent probability evidence.
            semantic_confidence=observation.semantic_confidence,
            direction=observation.direction,
            current_support_position=observation.support_position,
            last_reliable_support=history,
            formation_time=formation,
        )
        self._records[observation.trace_id] = record
        self._observations[observation.observation_id] = observation
        return record
