from enum import Enum
from typing import Any, Dict, Optional


def class2dict(
    object: object,
    class_key: Optional[str] = None,
) -> Dict[str, Any]:
    """Format class object to dict."""
    if isinstance(object, dict):
        data = {}
        for key, value in object.items():
            data[key] = class2dict(value, class_key)
        return data
    elif isinstance(object, Enum):
        return str(object)
    elif hasattr(object, "_ast"):
        return class2dict(object._ast())
    elif hasattr(object, "__iter__") and not isinstance(object, str):
        return [class2dict(v, class_key) for v in object]
    elif hasattr(object, "__dict__"):
        data = dict(
            [
                (key, class2dict(value, class_key))
                for key, value in object.__dict__.items()
                if not callable(value) and not key.startswith("_")
            ]
        )
        if class_key is not None and hasattr(object, "__class__"):
            data[class_key] = object.__class__.__name__
        return data
    else:
        return data
