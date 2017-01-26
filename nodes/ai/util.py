# util.py

def enum(**enums):
    return type('Enum', (), enums)
