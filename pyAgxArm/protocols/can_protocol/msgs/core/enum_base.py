from enum import IntEnum, Enum

class IntEnumBase(IntEnum):
    def __str__(self):
        # return f"{self.__class__.__name__}.{self.name}(0x{self.value:X})"
        return f"{self.name}(0x{self.value:X})"
    def __repr__(self):
        # return f"{self.__class__.__name__}.{self.name}(0x{self.value:X})"
        return f"{self.name}(0x{self.value:X})"
    @classmethod
    def match_value(cls, val):
        if not isinstance(val, int):
            raise ValueError(f"{cls.__name__}: input value must be an integer, got {type(val).__name__}")
        try:
            return cls(val)
        except ValueError:
            if hasattr(cls, "UNKNOWN"):
                return cls.UNKNOWN
            else:
                raise ValueError(f"{cls.__name__}: invalid enum value 0x{val:X}, and no UNKNOWN defined")
    @classmethod
    def value_list(cls):
        return [e.value for e in cls]
    

class EnumBase(Enum):
    def __str__(self):
        return f"{self.name}({self.value})"
    def __repr__(self):
        return f"{self.name}({self.value})"
    @classmethod
    def match_value(cls, val):
        try:
            return cls(val)
        except ValueError:
            if hasattr(cls, "UNKNOWN"):
                return cls.UNKNOWN
            else:
                raise ValueError(f"{cls.__name__}: invalid enum value {val}, and no UNKNOWN defined")
    @classmethod
    def value_list(cls):
        return [e.value for e in cls]
