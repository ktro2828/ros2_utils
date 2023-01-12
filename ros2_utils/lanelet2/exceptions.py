class AppException(Exception):
    error_code = ""

    def __init__(self, message=None):
        if message is not None:
            self.message = message

    def to_dict(self):
        return {"message": self.message}


class InvalidFileFormatError(AppException):
    message = "OSM file does not have the correct format."


class MissingResourceError(AppException):
    message = "File not found."


class InvalidMapDateError(AppException):
    message = "Map data is not correct."
