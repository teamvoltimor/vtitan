from buf.validate import validate_pb2 as _validate_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from collections.abc import Iterable as _Iterable, Mapping as _Mapping
from typing import ClassVar as _ClassVar, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class ExportFormat(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    EXPORT_FORMAT_UNSPECIFIED: _ClassVar[ExportFormat]
    EXPORT_FORMAT_SEGMENTATION: _ClassVar[ExportFormat]
    EXPORT_FORMAT_DETECTION: _ClassVar[ExportFormat]
EXPORT_FORMAT_UNSPECIFIED: ExportFormat
EXPORT_FORMAT_SEGMENTATION: ExportFormat
EXPORT_FORMAT_DETECTION: ExportFormat

class Point(_message.Message):
    __slots__ = ("x", "y")
    X_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    x: float
    y: float
    def __init__(self, x: _Optional[float] = ..., y: _Optional[float] = ...) -> None: ...

class ClickPoint(_message.Message):
    __slots__ = ("x", "y", "point_type", "class_name")
    X_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    POINT_TYPE_FIELD_NUMBER: _ClassVar[int]
    CLASS_NAME_FIELD_NUMBER: _ClassVar[int]
    x: float
    y: float
    point_type: str
    class_name: str
    def __init__(self, x: _Optional[float] = ..., y: _Optional[float] = ..., point_type: _Optional[str] = ..., class_name: _Optional[str] = ...) -> None: ...

class Shape(_message.Message):
    __slots__ = ("id", "class_name", "points")
    ID_FIELD_NUMBER: _ClassVar[int]
    CLASS_NAME_FIELD_NUMBER: _ClassVar[int]
    POINTS_FIELD_NUMBER: _ClassVar[int]
    id: str
    class_name: str
    points: _containers.RepeatedCompositeFieldContainer[Point]
    def __init__(self, id: _Optional[str] = ..., class_name: _Optional[str] = ..., points: _Optional[_Iterable[_Union[Point, _Mapping]]] = ...) -> None: ...

class SegmentRequest(_message.Message):
    __slots__ = ("image_id", "image_path", "points", "class_names")
    IMAGE_ID_FIELD_NUMBER: _ClassVar[int]
    IMAGE_PATH_FIELD_NUMBER: _ClassVar[int]
    POINTS_FIELD_NUMBER: _ClassVar[int]
    CLASS_NAMES_FIELD_NUMBER: _ClassVar[int]
    image_id: int
    image_path: str
    points: _containers.RepeatedCompositeFieldContainer[ClickPoint]
    class_names: _containers.RepeatedScalarFieldContainer[str]
    def __init__(self, image_id: _Optional[int] = ..., image_path: _Optional[str] = ..., points: _Optional[_Iterable[_Union[ClickPoint, _Mapping]]] = ..., class_names: _Optional[_Iterable[str]] = ...) -> None: ...

class SegmentResponse(_message.Message):
    __slots__ = ("state", "message", "shapes")
    STATE_FIELD_NUMBER: _ClassVar[int]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    SHAPES_FIELD_NUMBER: _ClassVar[int]
    state: str
    message: str
    shapes: _containers.RepeatedCompositeFieldContainer[Shape]
    def __init__(self, state: _Optional[str] = ..., message: _Optional[str] = ..., shapes: _Optional[_Iterable[_Union[Shape, _Mapping]]] = ...) -> None: ...

class AugmentedImage(_message.Message):
    __slots__ = ("path", "format_used", "parent_id")
    PATH_FIELD_NUMBER: _ClassVar[int]
    FORMAT_USED_FIELD_NUMBER: _ClassVar[int]
    PARENT_ID_FIELD_NUMBER: _ClassVar[int]
    path: str
    format_used: ExportFormat
    parent_id: int
    def __init__(self, path: _Optional[str] = ..., format_used: _Optional[_Union[ExportFormat, str]] = ..., parent_id: _Optional[int] = ...) -> None: ...

class JobProgress(_message.Message):
    __slots__ = ("status", "message", "stage", "progress", "details", "finished", "error", "augmented")
    STATUS_FIELD_NUMBER: _ClassVar[int]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    STAGE_FIELD_NUMBER: _ClassVar[int]
    PROGRESS_FIELD_NUMBER: _ClassVar[int]
    DETAILS_FIELD_NUMBER: _ClassVar[int]
    FINISHED_FIELD_NUMBER: _ClassVar[int]
    ERROR_FIELD_NUMBER: _ClassVar[int]
    AUGMENTED_FIELD_NUMBER: _ClassVar[int]
    status: str
    message: str
    stage: str
    progress: float
    details: str
    finished: bool
    error: str
    augmented: AugmentedImage
    def __init__(self, status: _Optional[str] = ..., message: _Optional[str] = ..., stage: _Optional[str] = ..., progress: _Optional[float] = ..., details: _Optional[str] = ..., finished: _Optional[bool] = ..., error: _Optional[str] = ..., augmented: _Optional[_Union[AugmentedImage, _Mapping]] = ...) -> None: ...

class AugmentSource(_message.Message):
    __slots__ = ("image_id", "path", "format_used")
    IMAGE_ID_FIELD_NUMBER: _ClassVar[int]
    PATH_FIELD_NUMBER: _ClassVar[int]
    FORMAT_USED_FIELD_NUMBER: _ClassVar[int]
    image_id: int
    path: str
    format_used: ExportFormat
    def __init__(self, image_id: _Optional[int] = ..., path: _Optional[str] = ..., format_used: _Optional[_Union[ExportFormat, str]] = ...) -> None: ...

class AugmentRequest(_message.Message):
    __slots__ = ("sources", "num_augmentations")
    SOURCES_FIELD_NUMBER: _ClassVar[int]
    NUM_AUGMENTATIONS_FIELD_NUMBER: _ClassVar[int]
    sources: _containers.RepeatedCompositeFieldContainer[AugmentSource]
    num_augmentations: int
    def __init__(self, sources: _Optional[_Iterable[_Union[AugmentSource, _Mapping]]] = ..., num_augmentations: _Optional[int] = ...) -> None: ...

class TrainRequest(_message.Message):
    __slots__ = ("model_name", "epochs", "batch", "imgsz", "data_yaml_path")
    MODEL_NAME_FIELD_NUMBER: _ClassVar[int]
    EPOCHS_FIELD_NUMBER: _ClassVar[int]
    BATCH_FIELD_NUMBER: _ClassVar[int]
    IMGSZ_FIELD_NUMBER: _ClassVar[int]
    DATA_YAML_PATH_FIELD_NUMBER: _ClassVar[int]
    model_name: str
    epochs: int
    batch: int
    imgsz: int
    data_yaml_path: str
    def __init__(self, model_name: _Optional[str] = ..., epochs: _Optional[int] = ..., batch: _Optional[int] = ..., imgsz: _Optional[int] = ..., data_yaml_path: _Optional[str] = ...) -> None: ...
