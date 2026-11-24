"""ROS names on the C → A/B and B ↔ A boundaries. No ROS imports."""

PERSON_MASKS = "/search/person_masks"
BEARING_EVIDENCE = "/search/bearing_evidence"
REVIEW_GOAL = "/search/review_goal"
REVIEW_CANCEL = "/search/review_cancel"
REVIEW_STATUS = "/search/review_status"


def camera_topic(drone, stream, channel):
    return f"/drone_{drone}/{stream}/{channel}"


def view_execution(drone):
    return f"/drone_{drone}/view_execution"


def target_observations(robot):
    return f"/{robot}/search/target_observations"
