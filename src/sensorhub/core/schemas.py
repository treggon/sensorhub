
from pydantic import BaseModel, Field
from typing import Optional, Any, List
from datetime import datetime

class SensorInfo(BaseModel):
    id: str
    kind: str
    description: Optional[str] = None

class Sample(BaseModel):
    sensor_id: str
    ts: datetime = Field(..., description="UTC timestamp when sample was captured")
    data: Any

class HistoryResponse(BaseModel):
    sensor_id: str
    samples: List[Sample]
