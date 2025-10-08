# api.py
from fastapi import APIRouter
from telemetry_payload import build_tracker_payload
from geo_15 import state, get_latest_base, get_latest_drone  # reuse your functions/instance

router = APIRouter(prefix="/api")

@router.get("/telemetry")
def get_telemetry():
    return build_tracker_payload(
        state=state,
        base_sample=get_latest_base(),
        drone_sample=get_latest_drone()
    )
