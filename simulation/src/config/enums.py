from enum import Enum

class Section(Enum):
    """Track section enumeration"""
    NORTH = 'north'
    SOUTH = 'south'
    EAST = 'east'
    WEST = 'west'

    def __str__(self):
        """Return string value for compatibility"""
        return self.value

    @classmethod
    def from_string(cls, value):
        """Create enum from string"""
        value_lower = value.lower()
        for section in cls:
            if section.value == value_lower:
                return section
        raise ValueError(f"Invalid section: {value}")

    @property
    def capitalized(self):
        """Return capitalized name"""
        return self.value.capitalize()


class Direction(Enum):
    """Starting direction enumeration"""
    CLOCKWISE = 'clockwise'
    COUNTERCLOCKWISE = 'counterclockwise'

    def __str__(self):
        """Return string value for compatibility"""
        return self.value

    @classmethod
    def from_string(cls, value):
        """Create enum from string"""
        value_lower = value.lower()
        for direction in cls:
            if direction.value == value_lower:
                return direction
        raise ValueError(f"Invalid direction: {value}")