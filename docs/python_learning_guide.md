# Python Learning Guide for Team 3164

A progressive guide to learn Python by working with our robot code. Each section builds on the previous one.

---

## How to Use This Guide

1. Read each section in order
2. Find the example in our actual code (file paths provided)
3. Try modifying small things to see what happens
4. Complete the practice exercises at the end
5. Move to the next section when you're comfortable

**Pro tip:** Open the referenced files side-by-side with this guide!

---

# Part 1: The Basics

## 1.1 Variables - Storing Information

Variables are names that hold values. Think of them as labeled boxes where you store things.

**Why variables matter:** Instead of typing `78.5` everywhere for the max elevator height, we store it once in a variable called `MAX_HEIGHT`. If we need to change it, we only change it in one place!

```python
# Numbers (integers and decimals)
motor_id = 9                    # Integer (whole number) - no decimal point
max_height = 78.5               # Float (decimal number) - has a decimal point
negative_num = -15              # Negative numbers work too

# Text (strings) - always in quotes
camera_name = "front_camera"    # Double quotes
team_name = 'Stealth Tigers'    # Single quotes work too (pick one style)
empty_string = ""               # Empty string is valid

# True/False (booleans) - only two possible values
is_enabled = True               # Note: capital T
use_auto = False                # Note: capital F
# Booleans are used for yes/no, on/off, true/false decisions

# None - represents "nothing" or "no value"
current_target = None           # We haven't set a target yet
```

**Naming rules:**
- Must start with a letter or underscore (`_`)
- Can contain letters, numbers, and underscores
- Case sensitive (`speed` and `Speed` are different!)
- Use `snake_case` for variables (words separated by underscores)

```python
# Good variable names
motor_speed = 0.5
elevator_height = 45.0
is_at_target = True

# Bad variable names (but technically valid)
x = 0.5                 # Too vague - what is x?
MotorSpeed = 0.5        # Should be snake_case, not CamelCase
2nd_motor = 9           # Can't start with a number - ERROR!
```

**In our code:** Look at `src/constants.py` lines 41-50 for motor IDs. Notice how each constant has a clear, descriptive name.

---

## 1.2 Basic Math

Python can do all the math you need for robotics calculations.

```python
# Basic arithmetic
total = 5 + 3           # Addition: 8
difference = 10 - 4     # Subtraction: 6
product = 3 * 4         # Multiplication: 12
quotient = 20 / 4       # Division: 5.0 (always returns a float!)
integer_div = 20 // 4   # Integer division: 5 (drops the decimal)
remainder = 17 % 5      # Modulo (remainder): 2 (17 ÷ 5 = 3 remainder 2)
power = 2 ** 3          # Exponent (2^3): 8

# Order of operations (PEMDAS applies!)
result = 2 + 3 * 4      # 14, not 20 (multiplication first)
result = (2 + 3) * 4    # 20 (parentheses first)

# Compound assignment (shortcuts)
speed = 10
speed = speed + 5       # Long way: speed is now 15
speed += 5              # Short way: speed is now 20
speed -= 3              # speed is now 17
speed *= 2              # speed is now 34
speed /= 2              # speed is now 17.0

# Useful built-in functions
absolute = abs(-5)              # Absolute value: 5
smallest = min(3, 7, 2)         # Minimum: 2
largest = max(3, 7, 2)          # Maximum: 7
rounded = round(3.14159, 2)     # Round to 2 decimals: 3.14
power = pow(2, 3)               # Same as 2 ** 3: 8

# Converting between types
int_val = int(3.7)              # Convert to integer: 3 (truncates, doesn't round!)
float_val = float(5)            # Convert to float: 5.0
str_val = str(42)               # Convert to string: "42"
```

**Robotics examples:**
```python
# Calculate wheel RPM from motor RPM with gear ratio
motor_rpm = 5000
gear_ratio = 8.14
wheel_rpm = motor_rpm / gear_ratio  # About 614 RPM

# Convert inches to meters (we do this a lot!)
height_inches = 78.5
height_meters = height_inches * 0.0254  # About 1.99 meters
```

**In our code:** See `src/container.py` line 69 for `abs()` and `**` usage in joystick processing.

---

## 1.3 Printing and Seeing Output

Print statements help you debug by showing values while code runs.

```python
# Basic print
print("Hello!")                         # Output: Hello!
print(42)                               # Output: 42
print(True)                             # Output: True

# Print multiple things
motor_id = 9
print("Motor ID is:", motor_id)         # Output: Motor ID is: 9
print("A", "B", "C")                     # Output: A B C (space-separated)

# F-strings (formatted strings) - THE BEST WAY!
# Put 'f' before the quote, then use {variable} inside
height = 45.5
name = "elevator"
print(f"The {name} height is {height} inches")
# Output: The elevator height is 45.5 inches

# F-strings can do math and call functions
radius = 5
print(f"Area: {3.14159 * radius ** 2}")     # Output: Area: 78.53975
print(f"Absolute: {abs(-10)}")               # Output: Absolute: 10

# Formatting numbers in f-strings
pi = 3.14159265359
print(f"Pi is {pi:.2f}")                # Output: Pi is 3.14 (2 decimal places)
print(f"Pi is {pi:.4f}")                # Output: Pi is 3.1416 (4 decimal places)

big_num = 1234567
print(f"Count: {big_num:,}")            # Output: Count: 1,234,567 (with commas)

# Older formatting methods (you'll see these in other code)
print("Height: " + str(height))         # String concatenation
print("Height: %s" % height)            # Old % formatting
print("Height: {}".format(height))      # .format() method
# PREFER f-strings - they're cleaner and faster!
```

**In our code:** We use SmartDashboard instead of print for robot display, but f-strings are used throughout for formatting. See `src/vision.py` line 37.

---

## 1.4 Comments - Notes in Code

Comments are notes for humans. Python ignores them completely.

```python
# This is a single-line comment - starts with #
motor_id = 9  # Comments can go at the end of a line too

# Use comments to explain WHY, not WHAT
# BAD: Set motor_id to 9
# GOOD: Left elevator motor - see wiring diagram

# Multi-line comments use triple quotes
"""
This is a multi-line comment (actually a docstring).
Use it for longer explanations or to temporarily
disable blocks of code during testing.
"""

'''
Single quotes work too for multi-line comments,
but double quotes are more common.
'''

# Temporarily disable code with comments
# print("This won't run")
print("This will run")

# TODO comments mark things to fix later
# TODO: Add error handling here
# FIXME: This calculation seems wrong
# NOTE: This assumes positive values only
```

**Docstrings - Special Comments:**
```python
def calculate_speed(distance, time):
    """
    Calculate speed from distance and time.

    Args:
        distance: The distance traveled in meters
        time: The time taken in seconds

    Returns:
        Speed in meters per second
    """
    return distance / time
```

**In our code:** See comments throughout `src/swerve_config.py` explaining motor configurations.

---

# Part 2: Making Decisions

## 2.1 If Statements

If statements let your code make decisions based on conditions.

**CRITICAL: Indentation matters in Python!** Code inside an if block MUST be indented (usually 4 spaces). This is how Python knows what code belongs to the if statement.

```python
height = 50

# Basic if - runs the indented code only if condition is True
if height > 40:
    print("High position")      # This is inside the if
    print("Be careful!")        # This is also inside the if
print("This always runs")       # This is outside - no indent

# If-else - do one thing or another
if height > 40:
    print("High position")
else:
    print("Low position")

# If-elif-else - check multiple conditions in order
# Python checks from top to bottom, runs the FIRST match, then stops
if height > 70:
    print("Level 4")        # Only if height > 70
elif height > 40:
    print("Level 3")        # Only if height > 40 AND height <= 70
elif height > 30:
    print("Level 2")        # Only if height > 30 AND height <= 40
else:
    print("Level 1")        # If none of the above matched

# You can have multiple elif blocks
# You don't need an else block if you don't want one
if height > 70:
    print("Very high")
elif height > 50:
    print("Medium high")
# If height <= 50, nothing happens (no else)
```

**Nested if statements:**
```python
has_piece = True
at_target = True

if has_piece:
    print("We have a game piece")
    if at_target:
        print("Ready to score!")
    else:
        print("Moving to target...")
else:
    print("Need to pick up a piece")
```

**In our code:** See `src/robot.py` lines 20-21 for checking if autonomous command exists before scheduling it.

---

## 2.2 Comparisons and Boolean Logic

Comparisons create True/False values that control if statements.

```python
# Comparison operators - all return True or False
x = 10
y = 5

x == y      # Equal to: False (10 is not equal to 5)
x != y      # Not equal to: True (10 is not equal to 5)
x > y       # Greater than: True (10 > 5)
x < y       # Less than: False (10 is not less than 5)
x >= y      # Greater than or equal: True
x <= y      # Less than or equal: False

# COMMON MISTAKE: = vs ==
x = 5       # Assignment: puts 5 into x
x == 5      # Comparison: checks if x equals 5, returns True/False

# Comparing strings
name = "elevator"
name == "elevator"      # True (exact match)
name == "Elevator"      # False (case sensitive!)
name.lower() == "elevator"  # True (convert to lowercase first)

# Boolean operators - combine multiple conditions
height = 50
speed = 0.5

# AND - both must be true
if height > 40 and speed < 1.0:
    print("High and slow")          # This runs

# OR - at least one must be true
if height > 70 or speed > 1.0:
    print("Either very high or fast")  # This doesn't run

# NOT - flips True to False and vice versa
is_disabled = False
if not is_disabled:
    print("System is enabled")      # This runs

# Combining multiple conditions
if (height > 30 and height < 60) or speed == 0:
    print("In safe range or stopped")

# Chained comparisons (Python special!)
if 30 < height < 60:        # Same as: height > 30 and height < 60
    print("In range")
```

**Truthy and Falsy values:**
```python
# These are all "falsy" (treated as False in if statements):
if 0:           # Zero is falsy
if "":          # Empty string is falsy
if []:          # Empty list is falsy
if None:        # None is falsy
if False:       # Obviously False is falsy

# These are all "truthy" (treated as True):
if 1:           # Any non-zero number
if "hello":     # Any non-empty string
if [1, 2, 3]:   # Any non-empty list
if True:        # Obviously True

# Useful for checking if something exists
name = ""
if name:                    # Falsy because empty string
    print(f"Hello {name}")
else:
    print("No name provided")
```

**In our code:** See `src/container.py` lines 117-121 for complex boolean conditions with `and`.

---

## 2.3 Quick If (Ternary Operator)

A one-line shortcut for simple if-else statements.

```python
# Long way
speed = 0.8
if speed > 0:
    direction = 1
else:
    direction = -1

# Short way (ternary operator)
# Format: value_if_true if condition else value_if_false
direction = 1 if speed > 0 else -1

# More examples
status = "enabled" if is_active else "disabled"
message = "High" if height > 50 else "Low"

# Can be used anywhere you need a value
print(f"Direction: {1 if speed > 0 else -1}")

# Can be chained (but don't go crazy - it gets hard to read!)
level = "high" if height > 70 else "medium" if height > 40 else "low"
# Better to use regular if-elif-else for complex logic
```

**In our code:** See `src/container.py` lines 533-534:
```python
def sgn(x):
    return 1 if x > 0 else -1
```
This returns the sign of a number: 1 for positive, -1 for negative.

---

# Part 3: Collections - Storing Multiple Things

## 3.1 Lists - Ordered Collections

Lists hold multiple items in order. Think of them as a numbered container.

```python
# Creating lists - use square brackets []
heights = [30.5, 31, 29.75, 45.25, 78.5]
motor_ids = [9, 10, 11, 12]
names = ["elevator", "arm", "claw"]
mixed = [1, "hello", True, 3.14]    # Can mix types (but usually don't)
empty_list = []                      # Empty list

# Accessing items by index (position)
# IMPORTANT: Counting starts at 0, not 1!
# Index:      0      1      2       3      4
heights = [30.5,   31,  29.75,  45.25,  78.5]

first = heights[0]          # 30.5 (first item)
second = heights[1]         # 31 (second item)
last = heights[-1]          # 78.5 (last item - negative counts from end)
second_last = heights[-2]   # 45.25

# Slicing - get a portion of the list
heights[1:3]                # [31, 29.75] (index 1 up to but not including 3)
heights[:3]                 # [30.5, 31, 29.75] (first 3 items)
heights[2:]                 # [29.75, 45.25, 78.5] (from index 2 to end)
heights[::2]                # [30.5, 29.75, 78.5] (every 2nd item)

# Modifying lists
heights[0] = 32.0           # Change first item
heights.append(80.0)        # Add to end: [30.5, 31, 29.75, 45.25, 78.5, 80.0]
heights.insert(0, 25.0)     # Insert at index 0: [25.0, 30.5, ...]
heights.pop()               # Remove and return last item: 80.0
heights.pop(0)              # Remove and return item at index 0: 25.0
heights.remove(31)          # Remove first occurrence of value 31

# List info
length = len(heights)       # How many items: 5
is_in = 45.25 in heights    # Check if value exists: True
index = heights.index(45.25) # Find index of value: 3

# Sorting
numbers = [3, 1, 4, 1, 5, 9]
numbers.sort()              # Sort in place: [1, 1, 3, 4, 5, 9]
numbers.sort(reverse=True)  # Sort descending: [9, 5, 4, 3, 1, 1]
sorted_copy = sorted(numbers)  # Return new sorted list, original unchanged

# Combining lists
list1 = [1, 2, 3]
list2 = [4, 5, 6]
combined = list1 + list2    # [1, 2, 3, 4, 5, 6]
list1.extend(list2)         # Modify list1 in place
```

**Common list patterns in robotics:**
```python
# Store sensor readings
readings = []
readings.append(sensor.getValue())  # Add new reading
if len(readings) > 10:
    readings.pop(0)  # Keep only last 10 readings
average = sum(readings) / len(readings)
```

**In our code:** See `src/vision.py` lines 15-21 for building a list of pose estimators.

---

## 3.2 Tuples - Fixed Collections

Tuples are like lists but cannot be changed after creation. Use parentheses `()`.

```python
# Creating tuples
position = (17.75, 29.75)           # (x, y) position
rgb_color = (255, 0, 0)             # Red color (R, G, B)
single_item = (42,)                 # Single item tuple needs comma!
no_parens = 1, 2, 3                 # Parentheses are optional

# Accessing items (same as lists)
x = position[0]                     # 17.75
y = position[1]                     # 29.75

# But you CANNOT modify tuples
# position[0] = 20.0                # ERROR! Tuples are immutable

# Why use tuples?
# 1. They're faster than lists
# 2. They signal "this shouldn't change"
# 3. They can be used as dictionary keys (lists can't)
# 4. Functions can return multiple values as tuples

# Tuple unpacking - super useful!
position = (17.75, 29.75)
x, y = position                     # x=17.75, y=29.75

rgb = (255, 128, 0)
red, green, blue = rgb              # Unpack all three

# Swap values without a temp variable
a = 1
b = 2
a, b = b, a                         # Now a=2, b=1 (magic!)

# Ignore values with underscore
x, _, z = (1, 2, 3)                 # We don't care about the middle value

# Unpack with * for variable length
first, *rest = [1, 2, 3, 4, 5]      # first=1, rest=[2, 3, 4, 5]
first, *middle, last = [1, 2, 3, 4] # first=1, middle=[2, 3], last=4
```

**In our code:** See `src/swerve_config.py` lines 54-75 for the swerve modules tuple - we have exactly 4 modules and that won't change.

---

## 3.3 Dictionaries - Key-Value Pairs

Dictionaries store data with named keys instead of numbered positions.

```python
# Creating a dictionary - use curly braces {}
motor_config = {
    "id": 9,
    "name": "elevator_left",
    "inverted": True,
    "max_speed": 0.8
}

# Accessing values by key
motor_id = motor_config["id"]               # 9
motor_name = motor_config["name"]           # "elevator_left"

# KeyError if key doesn't exist!
# motor_config["missing"]                   # ERROR!

# Safe access with .get() - returns None if key missing
missing = motor_config.get("missing")       # None
missing = motor_config.get("missing", 0)    # 0 (default value)

# Adding/changing values
motor_config["speed"] = 0.5                 # Add new key
motor_config["id"] = 10                     # Change existing key

# Removing values
del motor_config["speed"]                   # Remove key
removed = motor_config.pop("inverted")      # Remove and return value

# Check if key exists
if "max_speed" in motor_config:
    print("Max speed is configured")

# Get all keys, values, or both
keys = motor_config.keys()                  # dict_keys(['id', 'name', ...])
values = motor_config.values()              # dict_values([9, 'elevator_left', ...])
items = motor_config.items()                # dict_items([('id', 9), ('name', ...)])

# Looping through dictionaries
for key in motor_config:
    print(f"{key}: {motor_config[key]}")

# Better way - unpack key and value together
for key, value in motor_config.items():
    print(f"{key}: {value}")

# Nested dictionaries (dictionaries inside dictionaries)
robot_config = {
    "elevator": {
        "left_motor": 9,
        "right_motor": 10
    },
    "arm": {
        "motor": 12
    }
}
elevator_left = robot_config["elevator"]["left_motor"]  # 9
```

**In our code:** See `src/constants.py` lines 136-149 for `CAMERAS` dictionary and lines 163-176 for `REEF_TRANSFORMATIONS`.

---

# Part 4: Loops - Repeating Actions

## 4.1 For Loops

For loops repeat code for each item in a collection.

```python
# Loop through a list
heights = [30.5, 31, 45.25, 78.5]
for height in heights:
    print(f"Height: {height}")
# Output:
# Height: 30.5
# Height: 31
# Height: 45.25
# Height: 78.5

# Loop with range() - generates numbers
for i in range(5):              # 0, 1, 2, 3, 4
    print(i)

for i in range(1, 6):           # 1, 2, 3, 4, 5 (start at 1, stop before 6)
    print(i)

for i in range(0, 10, 2):       # 0, 2, 4, 6, 8 (step by 2)
    print(i)

for i in range(5, 0, -1):       # 5, 4, 3, 2, 1 (count down)
    print(i)

# Loop with index using enumerate()
names = ["elevator", "arm", "claw"]
for index, name in enumerate(names):
    print(f"{index}: {name}")
# Output:
# 0: elevator
# 1: arm
# 2: claw

# Loop through dictionary
cameras = {"front": "camera1", "back": "camera2"}
for name, camera in cameras.items():
    print(f"{name}: {camera}")

# Loop through two lists together with zip()
names = ["elevator", "arm"]
ids = [9, 12]
for name, motor_id in zip(names, ids):
    print(f"{name} uses motor {motor_id}")

# Break - exit the loop early
for i in range(10):
    if i == 5:
        break           # Stop the loop
    print(i)            # Prints 0, 1, 2, 3, 4

# Continue - skip to next iteration
for i in range(5):
    if i == 2:
        continue        # Skip this iteration
    print(i)            # Prints 0, 1, 3, 4 (skips 2)
```

**In our code:** See `src/vision.py` line 16:
```python
for camera_name, transform in VisionConstants.CAMERAS.items():
```

---

## 4.2 While Loops

While loops repeat as long as a condition is true.

```python
# Basic while loop
count = 0
while count < 5:
    print(count)
    count += 1          # Don't forget this or infinite loop!
# Output: 0, 1, 2, 3, 4

# Loop until condition changes
searching = True
while searching:
    result = search_for_target()
    if result is not None:
        searching = False
        print("Found it!")

# While True with break (common pattern)
while True:
    user_input = input("Enter 'quit' to exit: ")
    if user_input == 'quit':
        break
    print(f"You entered: {user_input}")

# While with else (less common but useful)
attempts = 0
while attempts < 3:
    if try_connection():
        print("Connected!")
        break
    attempts += 1
else:
    # This runs if we exit normally (not via break)
    print("Failed to connect after 3 attempts")
```

**When to use for vs while:**
- Use `for` when you know how many times to loop (or looping through a collection)
- Use `while` when you don't know how many times - loop until something happens

---

## 4.3 List Comprehensions - Compact Loops

A shortcut for creating lists from loops. Once you get it, you'll love it!

```python
# Long way to create a list of squares
squares = []
for x in range(5):
    squares.append(x ** 2)
# Result: [0, 1, 4, 9, 16]

# Short way - list comprehension
squares = [x ** 2 for x in range(5)]
# Same result: [0, 1, 4, 9, 16]

# Format: [expression for item in iterable]

# More examples
numbers = [1, 2, 3, 4, 5]
doubled = [n * 2 for n in numbers]              # [2, 4, 6, 8, 10]
strings = [str(n) for n in numbers]             # ['1', '2', '3', '4', '5']

# With a condition (filter)
evens = [x for x in range(10) if x % 2 == 0]    # [0, 2, 4, 6, 8]
positives = [x for x in [-1, 2, -3, 4] if x > 0]  # [2, 4]

# With if-else (transform based on condition)
labels = ["even" if x % 2 == 0 else "odd" for x in range(5)]
# ['even', 'odd', 'even', 'odd', 'even']

# Nested loops in comprehension
pairs = [(x, y) for x in range(3) for y in range(3)]
# [(0,0), (0,1), (0,2), (1,0), (1,1), (1,2), (2,0), (2,1), (2,2)]

# Dictionary comprehension
ids = [9, 10, 11]
names = ["elevator", "arm", "claw"]
motor_dict = {name: id for name, id in zip(names, ids)}
# {'elevator': 9, 'arm': 10, 'claw': 11}
```

**In our code:** See `src/container.py` line 425:
```python
[chr(i) for i in range(ord('a'), ord('l') + 1)]
```
This creates `['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l']` for reef positions.

---

# Part 5: Functions - Reusable Code

## 5.1 Defining Functions

Functions are reusable blocks of code. Write once, use anywhere!

```python
# Basic function - no inputs, no output
def say_hello():
    print("Hello!")

# Call it
say_hello()             # Output: Hello!
say_hello()             # Can call as many times as you want

# Function with parameters (inputs)
def say_hello_to(name):
    print(f"Hello, {name}!")

say_hello_to("Team 3164")       # Output: Hello, Team 3164!
say_hello_to("Stealth Tigers")  # Output: Hello, Stealth Tigers!

# Multiple parameters
def introduce(name, role):
    print(f"{name} is the {role}")

introduce("Alex", "driver")     # Output: Alex is the driver

# Function that returns a value
def add(a, b):
    result = a + b
    return result       # Send the value back

sum_result = add(5, 3)  # sum_result = 8
print(add(10, 20))      # Output: 30

# Return immediately exits the function
def check_positive(num):
    if num < 0:
        return False    # Exit here if negative
    return True         # Only runs if num >= 0

# Return multiple values (as a tuple)
def get_position():
    x = 10.5
    y = 20.3
    return x, y         # Returns tuple (10.5, 20.3)

x, y = get_position()   # Unpack the returned tuple

# Return early for validation
def divide(a, b):
    if b == 0:
        print("Cannot divide by zero!")
        return None     # Return early with None
    return a / b
```

**In our code:** Functions are everywhere! Every method in every file is a function.

---

## 5.2 Default and Keyword Parameters

Make functions more flexible with default values and named arguments.

```python
# Default parameter values
def set_speed(speed=0.5):           # 0.5 is the default
    print(f"Speed set to {speed}")

set_speed()             # Uses default: Speed set to 0.5
set_speed(0.8)          # Override: Speed set to 0.8

# Multiple defaults
def configure_motor(id, speed=0.5, inverted=False):
    print(f"Motor {id}: speed={speed}, inverted={inverted}")

configure_motor(9)                          # Motor 9: speed=0.5, inverted=False
configure_motor(9, 0.8)                     # Motor 9: speed=0.8, inverted=False
configure_motor(9, 0.8, True)               # Motor 9: speed=0.8, inverted=True

# Keyword arguments - specify by name (order doesn't matter!)
configure_motor(9, inverted=True)           # Motor 9: speed=0.5, inverted=True
configure_motor(id=9, speed=0.8)            # Same as configure_motor(9, 0.8)
configure_motor(inverted=True, id=9)        # Order doesn't matter with keywords

# IMPORTANT: Non-default params must come before default params
def bad_func(a=1, b):       # ERROR! b has no default but comes after a
    pass

def good_func(b, a=1):      # OK - b (required) comes before a (optional)
    pass

# *args - accept any number of positional arguments
def sum_all(*numbers):
    total = 0
    for num in numbers:
        total += num
    return total

sum_all(1, 2)               # 3
sum_all(1, 2, 3, 4, 5)      # 15

# **kwargs - accept any number of keyword arguments
def print_config(**settings):
    for key, value in settings.items():
        print(f"{key}: {value}")

print_config(speed=0.5, inverted=True, id=9)
```

**In our code:** See `src/subsystems/elevator.py` line 170:
```python
def reset(self, height=ElevatorConstants.MINIMUM_CARRIAGE_HEIGHT):
```

---

## 5.3 Lambda Functions - Quick One-Liners

Lambda functions are small, anonymous (unnamed) functions for simple operations.

```python
# Regular function
def double(x):
    return x * 2

# Same thing as a lambda
double = lambda x: x * 2

# Format: lambda arguments: expression
# - Can have multiple arguments: lambda x, y: x + y
# - Can only have ONE expression (no statements like if/for blocks)
# - Automatically returns the expression result

# When to use lambdas:
# 1. Simple, one-line operations
# 2. When you need a quick function to pass to another function

# Common use: sorting with custom key
motors = [
    {"name": "elevator", "id": 9},
    {"name": "arm", "id": 12},
    {"name": "claw", "id": 11}
]
# Sort by id
sorted_motors = sorted(motors, key=lambda m: m["id"])
# Sort by name
sorted_motors = sorted(motors, key=lambda m: m["name"])

# Common use: with map() to transform a list
numbers = [1, 2, 3, 4, 5]
squared = list(map(lambda x: x ** 2, numbers))  # [1, 4, 9, 16, 25]

# Common use: with filter() to filter a list
numbers = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
evens = list(filter(lambda x: x % 2 == 0, numbers))  # [2, 4, 6, 8, 10]

# Lambda with no arguments
get_zero = lambda: 0
print(get_zero())       # 0

# Lambda with multiple arguments
add = lambda x, y: x + y
print(add(3, 5))        # 8
```

**In our code:** Lambdas are used heavily for button bindings! See `src/container.py` line 69-71:
```python
lambda: (abs(self.driver_joystick.forward()) ** self.speed_exponent) * sgn(self.driver_joystick.forward())
```
This lambda reads joystick input every time it's called.

---

# Part 6: Classes - Building Blocks

## 6.1 What is a Class?

A class is a blueprint for creating objects. Think of it like a cookie cutter - the class defines the shape, and objects are the actual cookies.

**Objects have:**
- **Attributes**: Data they store (like variables attached to the object)
- **Methods**: Things they can do (like functions attached to the object)

```python
# Define a class (the blueprint)
class Motor:
    # __init__ is the constructor - runs when you create a new Motor
    # 'self' is a reference to the object being created
    def __init__(self, motor_id, name):
        # These create attributes on the object
        self.id = motor_id          # Store the ID
        self.name = name            # Store the name
        self.speed = 0              # Start at speed 0
        self.enabled = True         # Start enabled

    # Methods - things the Motor can do
    def set_speed(self, speed):
        if self.enabled:
            self.speed = speed
            print(f"{self.name} speed set to {speed}")
        else:
            print(f"{self.name} is disabled!")

    def stop(self):
        self.speed = 0
        print(f"{self.name} stopped")

    def disable(self):
        self.enabled = False
        self.speed = 0

    def get_info(self):
        return f"Motor {self.id} ({self.name}): speed={self.speed}"


# Create motor objects (instances of the class)
left_motor = Motor(9, "Elevator Left")      # Calls __init__ with id=9, name="Elevator Left"
right_motor = Motor(10, "Elevator Right")   # Separate object with its own data

# Use them
left_motor.set_speed(0.5)       # "Elevator Left speed set to 0.5"
right_motor.set_speed(0.8)      # "Elevator Right speed set to 0.8"

left_motor.stop()               # "Elevator Left stopped"

# Access attributes directly
print(left_motor.speed)         # 0 (stopped)
print(right_motor.speed)        # 0.8 (still running)

# Each object has its own data
print(left_motor.name)          # "Elevator Left"
print(right_motor.name)         # "Elevator Right"
```

**Why use classes?**
- **Organization**: Group related data and functions together
- **Reusability**: Create multiple objects from one blueprint
- **Encapsulation**: Hide complexity inside the class

---

## 6.2 The `self` Parameter

`self` is how an object refers to itself. Every method needs `self` as its first parameter.

```python
class Elevator:
    def __init__(self):
        self.height = 0             # This elevator's height
        self.target = 0             # This elevator's target

    def get_height(self):
        return self.height          # Return THIS elevator's height

    def set_target(self, target):
        self.target = target        # Set THIS elevator's target
        print(f"Target set to {self.target}")

    def move_to_target(self):
        # Access both attributes using self
        if self.height < self.target:
            self.height += 1
            print(f"Moving up to {self.height}")
        elif self.height > self.target:
            self.height -= 1
            print(f"Moving down to {self.height}")
        else:
            print("At target!")

    def at_target(self):
        # Methods can call other methods via self
        return self.height == self.target


# When you call a method, Python passes 'self' automatically
elevator = Elevator()
elevator.set_target(50)     # Python does: Elevator.set_target(elevator, 50)
                            # 'self' becomes 'elevator'
```

**Common mistake:**
```python
class Broken:
    def __init__(self):
        height = 0              # WRONG! This is a local variable, not an attribute
        self.height = 0         # RIGHT! This creates an attribute

    def get_height(self):
        return height           # WRONG! 'height' doesn't exist
        return self.height      # RIGHT! Access attribute via self
```

**In our code:** Every method in our subsystems uses `self`. See `src/subsystems/elevator.py` for consistent examples.

---

## 6.3 Inheritance - Building on Other Classes

Create new classes based on existing ones. The child class gets everything from the parent, plus can add more.

```python
# Parent class (also called base class or superclass)
class Subsystem:
    def __init__(self, name):
        self.name = name
        self.enabled = True

    def enable(self):
        self.enabled = True
        print(f"{self.name} enabled")

    def disable(self):
        self.enabled = False
        print(f"{self.name} disabled")

    def periodic(self):
        # Meant to be overridden by child classes
        pass


# Child class (also called derived class or subclass)
# Syntax: class ChildClass(ParentClass):
class Elevator(Subsystem):
    def __init__(self):
        # Call parent's __init__ first!
        super().__init__("Elevator")    # Passes "Elevator" as name
        # Then add our own attributes
        self.height = 0
        self.target = 0

    def set_height(self, height):
        self.height = height

    # Override the parent's periodic method
    def periodic(self):
        print(f"Elevator at {self.height}, target {self.target}")


# Elevator has everything Subsystem has, plus more
elevator = Elevator()
elevator.enable()           # Inherited from Subsystem: "Elevator enabled"
elevator.set_height(50)     # Elevator's own method
elevator.periodic()         # Overridden method: "Elevator at 50, target 0"
print(elevator.name)        # Inherited attribute: "Elevator"

# Check inheritance
print(isinstance(elevator, Elevator))   # True
print(isinstance(elevator, Subsystem))  # True - Elevator IS a Subsystem
```

**Why use inheritance?**
- Avoid duplicating code
- Create a hierarchy of related classes
- Override behavior while keeping the rest

**In our code:** See `src/robot.py` line 9:
```python
class Robot(commands2.TimedCommandRobot):
```
Our Robot inherits from TimedCommandRobot, which provides all the robot lifecycle methods.

---

## 6.4 Inner Classes

Classes defined inside other classes. We use these for command classes that belong to a subsystem.

```python
class Elevator(Subsystem):
    def __init__(self):
        super().__init__()
        self.height = 0
        self.target = 0

    def set_height(self, height):
        self.height = height

    # Inner class - belongs to Elevator
    class SetHeightCommand(Command):
        def __init__(self, elevator, target_height):
            super().__init__()
            self.elevator = elevator        # Reference to the elevator
            self.target = target_height
            self.addRequirements(elevator)  # Tell the system we need the elevator

        def initialize(self):
            print(f"Starting move to {self.target}")

        def execute(self):
            # Move toward target each cycle
            if self.elevator.height < self.target:
                self.elevator.height += 1
            elif self.elevator.height > self.target:
                self.elevator.height -= 1

        def isFinished(self):
            return self.elevator.height == self.target

        def end(self, interrupted):
            if interrupted:
                print("Move was interrupted!")
            else:
                print("Move complete!")


# Usage - create command from the inner class
elevator = Elevator()
move_cmd = Elevator.SetHeightCommand(elevator, 50)

# Or access through the instance
move_cmd = elevator.SetHeightCommand(elevator, 50)
```

**Why inner classes?**
- Keeps related code together
- Makes it clear the command belongs to that subsystem
- Follows the Commands2 pattern

**In our code:** See inner command classes in `src/subsystems/elevator.py`, like `SetHeightCommand`.

---

# Part 7: Type Hints - Documenting Your Code

## 7.1 Basic Type Hints

Type hints tell others (and your editor) what types to expect. Python doesn't enforce them, but they're super helpful!

```python
# Variable type hints
motor_id: int = 9                   # This should be an integer
height: float = 45.5                # This should be a float
name: str = "elevator"              # This should be a string
enabled: bool = True                # This should be a boolean

# Function parameter type hints
def set_height(height: float) -> None:
    """Set the height. Returns nothing (None)."""
    print(f"Setting height to {height}")

# Return type hints (after the ->)
def get_height() -> float:
    """Returns the current height."""
    return 45.5

def is_at_target() -> bool:
    """Returns True if at target."""
    return True

def get_name() -> str:
    """Returns the subsystem name."""
    return "elevator"

# Multiple parameters with types
def configure(motor_id: int, speed: float, inverted: bool) -> None:
    pass

# Type hints for lists
def get_heights() -> list[float]:
    """Returns a list of floats."""
    return [30.5, 45.25, 78.5]

# Type hints for dictionaries
def get_config() -> dict[str, int]:
    """Returns a dict with string keys and int values."""
    return {"elevator": 9, "arm": 12}

# Type hints for tuples
def get_position() -> tuple[float, float]:
    """Returns an (x, y) position."""
    return (10.5, 20.3)
```

**Why use type hints?**
- Your editor can warn you about type mismatches
- Makes code easier to understand
- Documentation built into the code

---

## 7.2 Optional and Union Types

For when a value might be None or could be different types.

```python
from typing import Optional, Union

# Optional - the value might be None
def find_motor(motor_id: int) -> Optional[Motor]:
    """Returns a Motor if found, None if not found."""
    if motor_id in motors:
        return motors[motor_id]
    return None

# Usage - handle the possible None!
motor = find_motor(9)
if motor is not None:
    motor.set_speed(0.5)
# Or with a default
motor = find_motor(9) or default_motor

# Modern Python (3.10+) - use | instead of Optional
def find_motor(motor_id: int) -> Motor | None:
    pass

# Union - the value could be different types
def process_input(value: Union[int, float]) -> float:
    """Accepts either int or float."""
    return float(value)

# Modern Python (3.10+)
def process_input(value: int | float) -> float:
    return float(value)

# Optional with default
def set_speed(speed: float = 0.5) -> None:
    pass

# Class attributes with types
class Elevator:
    height: float
    target: float
    name: str

    def __init__(self):
        self.height = 0.0
        self.target = 0.0
        self.name = "elevator"
```

**In our code:** See `src/robot.py` line 16:
```python
self.autonomous_command: Optional[commands2.Command] = None
```

---

# Part 8: Robot-Specific Patterns

## 8.1 The Command Pattern

Our robot uses Commands2 - a framework where actions are represented as Command objects.

**Key concepts:**
- **Command**: An action the robot performs
- **Subsystem**: A part of the robot (elevator, arm, swerve)
- Commands "require" subsystems - only one command can use a subsystem at a time

```python
import commands2

# Instant command - runs once and finishes
# Good for: toggling states, resetting values
reset_cmd = commands2.InstantCommand(lambda: self.reset_gyro())

# Run command - runs repeatedly until interrupted
# Good for: continuous actions like driving
drive_cmd = commands2.RunCommand(
    lambda: self.drive(speed),      # execute() - runs every cycle
    self.swerve                     # requires this subsystem
)

# Wait command - waits for a duration
wait_cmd = commands2.WaitCommand(0.5)   # Wait 0.5 seconds

# Print command - prints a message (debugging)
print_cmd = commands2.PrintCommand("Hello!")

# Sequential - commands run one after another
auto_sequence = commands2.SequentialCommandGroup(
    drive_forward,      # First this
    turn_right,         # Then this
    drive_forward       # Finally this
)

# Parallel - commands run at the same time
# All must finish before the group finishes
parallel = commands2.ParallelCommandGroup(
    raise_elevator,     # These happen
    rotate_arm          # at the same time
)

# ParallelRace - run until ANY command finishes
# Good for: timeouts, "do this until that happens"
with_timeout = commands2.ParallelRaceGroup(
    slow_command,
    commands2.WaitCommand(5.0)  # Give up after 5 seconds
)

# ParallelDeadline - run until the FIRST command finishes
# Others are interrupted when deadline completes
deadline = commands2.ParallelDeadlineGroup(
    main_command,       # This is the deadline
    background_task     # This runs alongside but doesn't affect when we stop
)
```

**In our code:** See `src/container.py` lines 134-160 for command groups in autonomous routines.

---

## 8.2 Command Chaining

Commands can be chained with method calls - this is how we build complex behaviors.

```python
# andThen() - run another command after this one
move_cmd.andThen(score_cmd)

# beforeStarting() - run something before this command
score_cmd.beforeStarting(lambda: print("Starting score"))

# alongWith() - run another command at the same time
move_cmd.alongWith(prepare_cmd)

# withTimeout() - cancel if it takes too long
slow_cmd.withTimeout(2.0)       # Stop after 2 seconds

# until() - run until a condition is true
drive_cmd.until(lambda: self.at_target())

# onlyIf() - only run if condition is true
score_cmd.onlyIf(lambda: self.has_piece())

# unless() - run unless condition is true (opposite of onlyIf)
intake_cmd.unless(lambda: self.has_piece())

# Chaining multiple operations
final_cmd = (
    drive_to_target
    .andThen(commands2.WaitCommand(0.25))
    .andThen(score_piece)
    .withTimeout(5.0)
)

# Real example from our code
auto = (
    self.swerve.follow_path("Start to Reef")
    .andThen(commands2.WaitCommand(0.25))
    .andThen(self.score_coral())
    .withTimeout(2)
)
```

**In our code:** See `src/container.py` lines 134-146 for command chaining in autonomous.

---

## 8.3 Triggers - Button Bindings

Triggers connect controller buttons (or any condition) to commands.

```python
from commands2.button import Trigger

# Basic button triggers (from controller)
# onTrue - when button is pressed (rising edge)
self.driver.a_button.onTrue(reset_command)

# onFalse - when button is released (falling edge)
self.driver.a_button.onFalse(stop_command)

# whileTrue - run while button is held, stop when released
self.driver.b_button.whileTrue(intake_command)

# toggleOnTrue - toggle command on/off with each press
self.driver.x_button.toggleOnTrue(tracking_command)

# Create custom triggers from any condition
at_target_trigger = Trigger(lambda: self.elevator.at_target())
at_target_trigger.onTrue(commands2.PrintCommand("At target!"))

# Combine triggers with boolean logic
both_pressed = button_a.and_(button_b)      # Both must be true
either_pressed = button_a.or_(button_b)     # Either can be true
not_pressed = button_a.negate()             # Opposite of button state

# Complex condition trigger
ready_to_score = Trigger(
    lambda: self.elevator.at_target() and
            self.arm.at_target() and
            self.claw.has_piece()
)
ready_to_score.onTrue(self.led.set_green())

# Trigger with additional condition
self.driver.a_button.and_(
    Trigger(lambda: wpilib.DriverStation.isTeleop())
).whileTrue(drive_command)
```

**In our code:** See `src/container.py` lines 388-410 for button bindings.

---

## 8.4 Subsystem Structure

Every subsystem in our code follows this pattern:

```python
import commands2
from rev import SparkMax

class MySubsystem(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        # Initialize hardware (motors, sensors)
        self.motor = SparkMax(MOTOR_ID, SparkMax.MotorType.kBrushless)
        self.encoder = self.motor.getEncoder()

        # Initialize state variables
        self.target = 0.0
        self.enabled = True

    # --- Getter methods: read state ---
    def get_position(self) -> float:
        """Get current position from encoder."""
        return self.encoder.getPosition()

    def at_target(self) -> bool:
        """Check if we're at the target position."""
        return abs(self.get_position() - self.target) < 0.1

    def has_piece(self) -> bool:
        """Check if we're holding a game piece."""
        return self.sensor.get()

    # --- Setter methods: change state ---
    def set_speed(self, speed: float) -> None:
        """Set motor speed (-1 to 1)."""
        self.motor.set(speed)

    def set_target(self, target: float) -> None:
        """Set the target position."""
        self.target = target

    def stop(self) -> None:
        """Stop the motor."""
        self.motor.set(0)

    # --- periodic(): runs every 20ms ---
    def periodic(self) -> None:
        """Update dashboard, run control loops."""
        # Log values to SmartDashboard
        wpilib.SmartDashboard.putNumber("Position", self.get_position())
        wpilib.SmartDashboard.putBoolean("At Target", self.at_target())

    # --- Inner command classes ---
    class SetPositionCommand(commands2.Command):
        """Command to move to a position."""

        def __init__(self, subsystem, target: float):
            super().__init__()
            self.subsystem = subsystem
            self.target = target
            self.addRequirements(subsystem)     # We need exclusive access

        def initialize(self):
            """Called once when command starts."""
            self.subsystem.set_target(self.target)

        def execute(self):
            """Called every 20ms while running."""
            # Move toward target
            error = self.target - self.subsystem.get_position()
            self.subsystem.set_speed(error * 0.1)  # Simple P control

        def isFinished(self) -> bool:
            """Return True when command should end."""
            return self.subsystem.at_target()

        def end(self, interrupted: bool):
            """Called when command ends."""
            self.subsystem.stop()
            if interrupted:
                print("Command was interrupted!")
```

**In our code:** Study `src/subsystems/elevator.py` as the best example of this pattern.

---

# Part 9: Common Patterns in Our Code

## 9.1 Using Constants

All "magic numbers" go in `constants.py`. This makes them easy to find and change.

```python
# In constants.py
class ElevatorConstants:
    # Motor configuration
    LEFT_MOTOR_ID = 9
    RIGHT_MOTOR_ID = 10

    # Physical limits
    MAX_HEIGHT = 78.5       # inches
    MIN_HEIGHT = 30.5       # inches

    # Control parameters
    kP = 0.1
    kI = 0.0
    kD = 0.05
    TOLERANCE = 0.5         # inches

class ScoringHeights:
    LEVEL_1 = 31.0
    LEVEL_2 = 29.75
    LEVEL_3 = 45.25
    LEVEL_4 = 78.5
    LOADING = 30.5


# In other files - import and use
from constants import ElevatorConstants, ScoringHeights

class Elevator:
    def __init__(self):
        self.left_motor = SparkMax(ElevatorConstants.LEFT_MOTOR_ID)
        self.right_motor = SparkMax(ElevatorConstants.RIGHT_MOTOR_ID)

    def at_target(self) -> bool:
        error = abs(self.height - self.target)
        return error < ElevatorConstants.TOLERANCE

# Use scoring heights
elevator.set_target(ScoringHeights.LEVEL_3)
```

**Why constants?**
- One place to change values
- Clear names instead of mysterious numbers
- Easy to find all configurable values

---

## 9.2 getattr and setattr

Dynamic attribute access - when you don't know the attribute name until runtime.

```python
# Normal way - you know the attribute name
button = self.button_board.reef_a

# Dynamic way - attribute name comes from a variable
position = "a"
button = getattr(self.button_board, f"reef_{position}")
# This is the same as: self.button_board.reef_a

# Loop through multiple attributes
for position in ['a', 'b', 'c', 'd']:
    button = getattr(self.button_board, f"reef_{position}")
    button.onTrue(self.go_to_reef_position(position))

# setattr - set an attribute dynamically
setattr(self, "speed_exponent", 2)
# Same as: self.speed_exponent = 2

# Real example - toggle between values
current = getattr(self, "speed_mode")
new_value = "fast" if current == "slow" else "slow"
setattr(self, "speed_mode", new_value)

# hasattr - check if attribute exists
if hasattr(self.button_board, "reef_a"):
    print("reef_a button exists")
```

**In our code:** See `src/container.py` lines 425-426 for looping through reef position buttons.

---

## 9.3 Properties - Computed Attributes

Use `@property` for values that look like attributes but are calculated.

```python
class Climber:
    def __init__(self):
        self.encoder = Encoder()
        self._enabled = True

    # Property - getter
    @property
    def angle(self) -> float:
        """Get the current angle (calculated from encoder)."""
        return self.encoder.getPosition() * 360 / 4096  # Convert ticks to degrees

    # Property with setter
    @property
    def enabled(self) -> bool:
        return self._enabled

    @enabled.setter
    def enabled(self, value: bool):
        self._enabled = value
        if not value:
            self.stop()  # Stop motor when disabled


# Usage - looks like an attribute, but calls methods
climber = Climber()
print(climber.angle)        # Calls the getter, returns calculated angle
climber.enabled = False     # Calls the setter, which also stops the motor
```

**Why properties?**
- Calculate values on demand
- Add validation when setting values
- Keep the simple attribute syntax

**In our code:** See `src/subsystems/climber.py` lines 66-68 for the `angle` property.

---

## 9.4 Exception Handling

Handle errors gracefully instead of crashing.

```python
# Basic try-except
try:
    result = 10 / 0     # This will cause an error
except ZeroDivisionError:
    print("Cannot divide by zero!")
    result = 0

# Catch multiple exception types
try:
    value = int("not a number")
except ValueError:
    print("Invalid number format")
except TypeError:
    print("Wrong type provided")

# Catch any exception
try:
    risky_operation()
except Exception as e:
    print(f"Something went wrong: {e}")

# finally - always runs, even if there's an error
try:
    file = open("data.txt")
    data = file.read()
except FileNotFoundError:
    print("File not found")
finally:
    file.close()        # Always close the file

# Raising exceptions
def set_speed(speed: float):
    if speed < -1 or speed > 1:
        raise ValueError(f"Speed must be between -1 and 1, got {speed}")
    # ... rest of function

# Real example from our code
def current_command_name(self) -> str:
    try:
        return self.getCurrentCommand().getName()
    except AttributeError:
        return "None"   # No command is running
```

**In our code:** See `src/subsystems/coral_arm.py` lines 167-171 for try-except usage.

---

# Part 10: Quick Reference

## Common Operations

```python
# Math
abs(x)                  # Absolute value
min(a, b, c)            # Smallest value
max(a, b, c)            # Largest value
round(x, 2)             # Round to 2 decimal places
pow(x, y)               # x to the power of y
sum([1, 2, 3])          # Sum of a list: 6

# Strings
f"Value: {var}"         # F-string formatting
text.upper()            # "HELLO"
text.lower()            # "hello"
text.strip()            # Remove whitespace from ends
text.split(",")         # Split into list by comma
",".join(["a", "b"])    # Join list into string: "a,b"
text.replace("a", "b")  # Replace all "a" with "b"
text.startswith("Hi")   # Check if starts with "Hi"
len(text)               # Length of string

# Lists
items.append(x)         # Add to end
items.insert(0, x)      # Add at index 0
items.pop()             # Remove and return last
items.pop(0)            # Remove and return first
items.remove(x)         # Remove first occurrence of x
len(items)              # Number of items
x in items              # Check if x exists
items.index(x)          # Find index of x
items.sort()            # Sort in place
sorted(items)           # Return new sorted list
items.reverse()         # Reverse in place

# Dictionaries
d["key"]                # Get value (KeyError if missing)
d.get("key")            # Get value (None if missing)
d.get("key", default)   # Get value (default if missing)
d["key"] = value        # Set value
del d["key"]            # Remove key
"key" in d              # Check if key exists
d.keys()                # All keys
d.values()              # All values
d.items()               # All (key, value) pairs
d.update(other_dict)    # Merge another dict in

# Type checking
type(x)                 # Get type of x
isinstance(x, int)      # Check if x is an int
isinstance(x, (int, float))  # Check if x is int or float
```

## Key Files to Study

| What to Learn | File to Study |
|--------------|---------------|
| Robot lifecycle | `src/robot.py` |
| Subsystem pattern | `src/subsystems/elevator.py` |
| Button bindings | `src/container.py` |
| Constants organization | `src/constants.py` |
| Custom commands | `src/commands/swerve.py` |
| Type hints | `src/vision.py` |
| Inner command classes | `src/subsystems/coral_arm.py` |
| Field geometry | `src/field.py` |
| Swerve configuration | `src/swerve_config.py` |

---

# Practice Exercises

## Beginner Exercises (Variables, Math, Print)

### Exercise 1: Find the Constants
Open `src/constants.py` and find:
1. The motor ID for the claw
2. The maximum elevator height
3. The Level 3 scoring height
4. The gear ratio for the swerve drive modules

### Exercise 2: Calculate Conversions
Write a Python script that:
1. Converts 78.5 inches to meters (multiply by 0.0254)
2. Converts 45 degrees to radians (multiply by π/180)
3. Prints both results with f-strings showing 3 decimal places

### Exercise 3: Variable Types
Look at `src/constants.py` and identify one example each of:
1. An integer variable
2. A float variable
3. A boolean variable
4. A string variable

### Exercise 4: Print Debugging
Add a print statement to `src/subsystems/elevator.py` in the `periodic()` method that shows the current height. Run in simulation to see it.

---

## Intermediate Exercises (Conditionals, Loops, Functions)

### Exercise 5: Trace the Logic
In `src/container.py`, find the `sgn()` function (around line 533).
1. What does it return for input `5`?
2. What does it return for input `-3`?
3. What does it return for input `0`?

### Exercise 6: Understand Conditionals
Look at `src/robot.py` in the `autonomousInit()` method.
1. What condition is being checked?
2. What happens if the condition is True?
3. What happens if it's False?

### Exercise 7: Loop Through Motors
Write code that would loop through motor IDs 1-15 and print:
```
Motor 1: configured
Motor 2: configured
...
Motor 15: configured
```

### Exercise 8: List Comprehension
The code `[chr(i) for i in range(ord('a'), ord('l') + 1)]` appears in container.py.
1. What list does this create?
2. Write a similar comprehension that creates `['A', 'B', 'C', 'D', 'E', 'F']`

### Exercise 9: Dictionary Practice
Create a dictionary called `scoring_levels` that maps level names to heights:
- "loading" → 30.5
- "level1" → 31.0
- "level2" → 29.75
- "level3" → 45.25
- "level4" → 78.5

Then write code to:
1. Print the height for level3
2. Add a new entry "ground" → 25.0
3. Loop through and print all levels and heights

### Exercise 10: Write a Function
Write a function called `is_in_range(value, min_val, max_val)` that:
1. Takes three parameters
2. Returns True if value is between min_val and max_val (inclusive)
3. Returns False otherwise
4. Test it with: `is_in_range(50, 30, 78)` → should return True

---

## Advanced Exercises (Classes, Commands, Robot Patterns)

### Exercise 11: Trace Inheritance
Look at `src/robot.py`:
1. What class does `Robot` inherit from?
2. What methods does Robot override from its parent?
3. Find where `super().__init__()` would be called (or if it's needed)

### Exercise 12: Understand self
In `src/subsystems/elevator.py`:
1. Find three different attributes that use `self.`
2. Find a method that calls another method using `self.`
3. Explain why `self` is needed

### Exercise 13: Command Chaining
Look at `src/container.py` and find an example of command chaining (using `.andThen()`, `.withTimeout()`, etc.).
1. Write down the chain you found
2. Explain what each part does
3. What order do the commands execute in?

### Exercise 14: Button Binding
In `src/container.py`, find the `configure_button_bindings()` method.
1. Find a button that uses `.onTrue()`
2. Find a button that uses `.whileTrue()`
3. What's the difference between these two?

### Exercise 15: Modify a Constant
1. Find where scoring heights are defined in `constants.py`
2. What would you change to make Level 2 height 30.0 instead of 29.75?
3. What other files might be affected by this change?

### Exercise 16: Add a Button Binding
Plan out (don't actually code yet) how you would:
1. Add a binding for the Y button on the operator controller
2. Make it run an InstantCommand that prints "Y pressed!"
3. What file would you modify?
4. What method would you add the code to?

### Exercise 17: Create a Simple Subsystem
Design (on paper) a new subsystem called `LED` that:
1. Has a `color` attribute (string)
2. Has a `set_color(color)` method
3. Has a `get_color()` method that returns the current color
4. Has an inner command class `SetColorCommand`

### Exercise 18: Understand Triggers
In `src/container.py`, find where a `Trigger` is created with a lambda.
1. What condition does the trigger check?
2. What command does it run?
3. When does the command run (onTrue, whileTrue, etc.)?

### Exercise 19: Type Hints
Look at `src/vision.py`:
1. Find a function that returns `Optional[something]`
2. What does `Optional` mean?
3. Why might this function return `None`?

### Exercise 20: Full Trace
Trace what happens when the driver presses the A button during teleop:
1. Start in `src/container.py` - find the A button binding
2. What command does it trigger?
3. Follow that command - what subsystem(s) does it use?
4. What methods on those subsystems get called?

---

# Getting Help

- **Python Docs**: https://docs.python.org/3/
- **WPILib Python**: https://robotpy.readthedocs.io/
- **Commands2 Docs**: https://robotpy.readthedocs.io/projects/commands-v2/
- **Ask a mentor** - We're here to help!

Remember: The best way to learn is to read the existing code, make small changes, and see what happens. Don't be afraid to experiment in simulation!

---

# Glossary

| Term | Definition |
|------|------------|
| **Attribute** | A variable that belongs to an object (e.g., `self.height`) |
| **Boolean** | A True/False value |
| **Class** | A blueprint for creating objects |
| **Command** | An action the robot performs |
| **Dictionary** | A collection of key-value pairs |
| **Float** | A decimal number (e.g., 3.14) |
| **Function** | A reusable block of code |
| **Inheritance** | When a class is based on another class |
| **Instance** | An object created from a class |
| **Integer** | A whole number (e.g., 42) |
| **Lambda** | A small anonymous function |
| **List** | An ordered collection of items |
| **Method** | A function that belongs to a class |
| **Object** | An instance of a class |
| **Parameter** | An input to a function |
| **Property** | A method that acts like an attribute |
| **Return** | Send a value back from a function |
| **Self** | How an object refers to itself |
| **String** | Text data (e.g., "hello") |
| **Subsystem** | A part of the robot (elevator, arm, etc.) |
| **Trigger** | Connects a condition to a command |
| **Tuple** | An immutable ordered collection |
| **Type Hint** | Documentation of expected types |
| **Variable** | A name that holds a value |

---

# Answer Key

## Beginner Exercises

### Exercise 1: Find the Constants

**Answers:**

1. **Claw motor ID**: Look in `src/constants.py` for `ClawConstants` class
   ```python
   MOTOR_ID = 11
   ```

2. **Maximum elevator height**: Look in `ElevatorConstants` class
   ```python
   MAXIMUM_CARRIAGE_HEIGHT = 78.5  # inches
   ```

3. **Level 3 scoring height**: Look in `ScoringConstants` class
   ```python
   LEVEL_3_HEIGHT = 45.25  # inches
   ```

4. **Swerve drive gear ratio**: Look in `src/swerve_config.py`
   ```python
   DRIVE_PARAMS = TypicalDriveComponentParameters(
       wheel_circumference=...,
       gear_ratio=8.14,  # <-- This is it!
       ...
   )
   ```

**How to find things in constants.py:**
```
constants.py structure:
┌─────────────────────────────────────┐
│ class DrivingConstants:             │ ← General driving settings
│     MAX_VELOCITY = ...              │
│     USE_AUTO_SCORE = ...            │
├─────────────────────────────────────┤
│ class ElevatorConstants:            │ ← Elevator-specific
│     LEFT_MOTOR_ID = 9               │
│     RIGHT_MOTOR_ID = 10             │
│     MAXIMUM_CARRIAGE_HEIGHT = 78.5  │
├─────────────────────────────────────┤
│ class ClawConstants:                │ ← Claw-specific
│     MOTOR_ID = 11                   │
├─────────────────────────────────────┤
│ class ScoringConstants:             │ ← Scoring heights
│     LEVEL_1_HEIGHT = 31.0           │
│     LEVEL_2_HEIGHT = 29.75          │
│     LEVEL_3_HEIGHT = 45.25          │
│     LEVEL_4_HEIGHT = 78.5           │
└─────────────────────────────────────┘
```

---

### Exercise 2: Calculate Conversions

**Answer:**
```python
import math

# 1. Convert inches to meters
height_inches = 78.5
height_meters = height_inches * 0.0254
print(f"78.5 inches = {height_meters:.3f} meters")
# Output: 78.5 inches = 1.994 meters

# 2. Convert degrees to radians
angle_degrees = 45
angle_radians = angle_degrees * (math.pi / 180)
print(f"45 degrees = {angle_radians:.3f} radians")
# Output: 45 degrees = 0.785 radians
```

**Understanding the math:**
```
Inches to Meters:
┌──────────────────────────────────────────────┐
│  1 inch = 0.0254 meters (exactly)            │
│                                              │
│  78.5 inches × 0.0254 = 1.9939 meters        │
│                                              │
│  :.3f means "show 3 decimal places"          │
│  So we get: 1.994                            │
└──────────────────────────────────────────────┘

Degrees to Radians:
┌──────────────────────────────────────────────┐
│  A full circle = 360° = 2π radians           │
│                                              │
│  So: 1° = 2π/360 = π/180 radians             │
│                                              │
│  45° × (π/180) = 45 × 0.01745... = 0.785     │
│                                              │
│         90°                                  │
│          │                                   │
│   180° ──┼── 0°     (degrees)               │
│          │                                   │
│         270°                                 │
│                                              │
│         π/2                                  │
│          │                                   │
│     π ───┼── 0      (radians)               │
│          │                                   │
│        3π/2                                  │
└──────────────────────────────────────────────┘
```

---

### Exercise 3: Variable Types

**Answers from `src/constants.py`:**

1. **Integer example:**
   ```python
   LEFT_MOTOR_ID = 9     # No decimal point = integer
   ```

2. **Float example:**
   ```python
   MAXIMUM_CARRIAGE_HEIGHT = 78.5    # Has decimal point = float
   ```

3. **Boolean example:**
   ```python
   OPEN_LOOP = False     # True/False = boolean
   USE_AUTO_SCORE = True
   ```

4. **String example:**
   ```python
   # In the CAMERAS dictionary, the keys are strings:
   "front_right": Transform3d(...)
   ```

**Visual guide to types:**
```
┌─────────────────────────────────────────────────────────┐
│                    PYTHON DATA TYPES                    │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  INTEGER (int)         FLOAT (float)                    │
│  ┌─────────┐          ┌───────────┐                     │
│  │ 9       │          │ 78.5      │                     │
│  │ -15     │          │ 3.14159   │                     │
│  │ 0       │          │ -0.5      │                     │
│  │ 1000    │          │ 2.0       │  ← Still a float!   │
│  └─────────┘          └───────────┘                     │
│  No decimal            Has decimal                      │
│                                                         │
│  BOOLEAN (bool)        STRING (str)                     │
│  ┌─────────┐          ┌───────────────┐                 │
│  │ True    │          │ "hello"       │                 │
│  │ False   │          │ 'world'       │                 │
│  └─────────┘          │ "Motor 9"     │                 │
│  Only 2 values!       │ ""  ← empty   │                 │
│                       └───────────────┘                 │
│                       Always in quotes                  │
└─────────────────────────────────────────────────────────┘
```

---

### Exercise 4: Print Debugging

**Answer:**

Find the `periodic()` method in `src/subsystems/elevator.py` and add a print statement:

```python
def periodic(self) -> None:
    # Add this line:
    print(f"Elevator height: {self.carriage_height()}")

    # ... rest of existing code
```

**Why this works:**
```
Robot Loop (runs every 20ms = 50 times per second):
┌─────────────────────────────────────────────────┐
│                                                 │
│  ┌──────────────────┐                           │
│  │  robotPeriodic() │ ← Called by the system    │
│  └────────┬─────────┘                           │
│           │                                     │
│           ▼                                     │
│  ┌──────────────────┐                           │
│  │ CommandScheduler │                           │
│  │     .run()       │                           │
│  └────────┬─────────┘                           │
│           │                                     │
│           ▼                                     │
│  ┌──────────────────┐                           │
│  │ For each subsystem: │                        │
│  │   subsystem.periodic() │ ← YOUR PRINT HERE!  │
│  └──────────────────┘                           │
│                                                 │
│  This happens 50 times per second, so you'll   │
│  see LOTS of output! Consider adding a         │
│  condition to print less often.                │
└─────────────────────────────────────────────────┘
```

**Better version (prints less often):**
```python
def periodic(self) -> None:
    # Only print every ~1 second (50 cycles)
    if hasattr(self, '_print_counter'):
        self._print_counter += 1
    else:
        self._print_counter = 0

    if self._print_counter % 50 == 0:
        print(f"Elevator height: {self.carriage_height()}")
```

---

## Intermediate Exercises

### Exercise 5: Trace the Logic

**The `sgn()` function:**
```python
def sgn(x):
    return 1 if x > 0 else -1
```

**Answers:**

1. `sgn(5)` returns `1`
   - Is 5 > 0? YES → return 1

2. `sgn(-3)` returns `-1`
   - Is -3 > 0? NO → return -1

3. `sgn(0)` returns `-1`
   - Is 0 > 0? NO (0 is not greater than 0) → return -1

**Visual trace:**
```
sgn(5):
┌─────────────────────────────────┐
│  x = 5                          │
│                                 │
│  Is x > 0?                      │
│  Is 5 > 0?  ──────► YES         │
│                      │          │
│                      ▼          │
│              return 1           │
└─────────────────────────────────┘

sgn(-3):
┌─────────────────────────────────┐
│  x = -3                         │
│                                 │
│  Is x > 0?                      │
│  Is -3 > 0?  ──────► NO         │
│                      │          │
│                      ▼          │
│              return -1          │
└─────────────────────────────────┘

sgn(0):
┌─────────────────────────────────┐
│  x = 0                          │
│                                 │
│  Is x > 0?                      │
│  Is 0 > 0?  ──────► NO          │
│             (0 equals 0,        │
│              not greater than)  │
│                      │          │
│                      ▼          │
│              return -1          │
└─────────────────────────────────┘

Note: This function doesn't handle 0 specially!
A "true" sign function would return 0 for input 0.
```

---

### Exercise 6: Understand Conditionals

**In `src/robot.py`, the `autonomousInit()` method:**

```python
def autonomousInit(self) -> None:
    self.autonomous_command = self.container.get_autonomous_command()
    if self.autonomous_command:
        self.autonomous_command.schedule()
```

**Answers:**

1. **Condition being checked:** `if self.autonomous_command:`
   - This checks if `autonomous_command` is "truthy" (not None, not empty)

2. **If True:** `self.autonomous_command.schedule()` runs
   - The autonomous command starts executing

3. **If False:** Nothing happens
   - No `else` block, so the code just continues
   - The robot does nothing in autonomous

**Visual explanation:**
```
autonomousInit() flow:
┌────────────────────────────────────────────────────────┐
│                                                        │
│  1. Get the selected auto from SmartDashboard          │
│     ┌──────────────────────────────────────┐           │
│     │ self.autonomous_command =            │           │
│     │   self.container.get_autonomous_command() │      │
│     └──────────────────────────────────────┘           │
│                         │                              │
│                         ▼                              │
│  2. Check if we got a valid command                    │
│     ┌──────────────────────────────────────┐           │
│     │ if self.autonomous_command:          │           │
│     └──────────────────────────────────────┘           │
│                         │                              │
│            ┌────────────┴────────────┐                 │
│            │                         │                 │
│       Has command              No command              │
│       (truthy)                 (None/falsy)            │
│            │                         │                 │
│            ▼                         ▼                 │
│     ┌──────────────┐          ┌──────────────┐         │
│     │ .schedule()  │          │  Do nothing  │         │
│     │ Start auto!  │          │  Robot sits  │         │
│     └──────────────┘          └──────────────┘         │
│                                                        │
└────────────────────────────────────────────────────────┘

Why check for truthy?
┌────────────────────────────────────────────────────────┐
│ get_autonomous_command() might return:                 │
│                                                        │
│   - A Command object → truthy → runs the command       │
│   - None → falsy → nothing happens                     │
│                                                        │
│ Without the check, calling .schedule() on None would   │
│ crash the robot! The if statement prevents this.       │
└────────────────────────────────────────────────────────┘
```

---

### Exercise 7: Loop Through Motors

**Answer:**
```python
# Using range(1, 16) to go from 1 to 15
for motor_id in range(1, 16):
    print(f"Motor {motor_id}: configured")
```

**Output:**
```
Motor 1: configured
Motor 2: configured
Motor 3: configured
... (continues) ...
Motor 15: configured
```

**Understanding range():**
```
range(1, 16) generates: 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15
                        ↑                                               ↑
                      start                                     stop - 1

┌─────────────────────────────────────────────────────────────┐
│  range(start, stop) → generates numbers from start to stop-1│
│                                                             │
│  range(5)      → 0, 1, 2, 3, 4        (assumes start=0)     │
│  range(1, 5)   → 1, 2, 3, 4           (start=1, stop=5)     │
│  range(0, 10, 2) → 0, 2, 4, 6, 8      (step by 2)           │
│  range(5, 0, -1) → 5, 4, 3, 2, 1      (count backwards)     │
└─────────────────────────────────────────────────────────────┘

Loop execution:
┌────────────────────────────────────────────┐
│ Iteration 1:  motor_id = 1                 │
│               print("Motor 1: configured") │
├────────────────────────────────────────────┤
│ Iteration 2:  motor_id = 2                 │
│               print("Motor 2: configured") │
├────────────────────────────────────────────┤
│ ...                                        │
├────────────────────────────────────────────┤
│ Iteration 15: motor_id = 15                │
│               print("Motor 15: configured")│
├────────────────────────────────────────────┤
│ Loop ends (16 is not in range)             │
└────────────────────────────────────────────┘
```

---

### Exercise 8: List Comprehension

**Answers:**

1. **What the code creates:**
   ```python
   [chr(i) for i in range(ord('a'), ord('l') + 1)]
   # Creates: ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l']
   ```

2. **Comprehension for uppercase A-F:**
   ```python
   [chr(i) for i in range(ord('A'), ord('F') + 1)]
   # Creates: ['A', 'B', 'C', 'D', 'E', 'F']
   ```

**Understanding ord() and chr():**
```
┌─────────────────────────────────────────────────────────────┐
│  ord() = "ordinal" - converts character to ASCII number     │
│  chr() = "character" - converts ASCII number to character   │
│                                                             │
│  ord('a') = 97      chr(97) = 'a'                           │
│  ord('b') = 98      chr(98) = 'b'                           │
│  ord('A') = 65      chr(65) = 'A'                           │
│  ord('Z') = 90      chr(90) = 'Z'                           │
└─────────────────────────────────────────────────────────────┘

Breaking down the comprehension:
┌─────────────────────────────────────────────────────────────┐
│  [chr(i) for i in range(ord('a'), ord('l') + 1)]            │
│   ──┬──      ─┬─       ──────────┬───────────               │
│     │         │                  │                          │
│     │         │                  └─ range(97, 109)          │
│     │         │                     = 97,98,99...108        │
│     │         │                                             │
│     │         └─ i takes each value: 97, 98, 99, ...        │
│     │                                                       │
│     └─ chr(97)='a', chr(98)='b', chr(99)='c', ...           │
│                                                             │
│  Result: ['a','b','c','d','e','f','g','h','i','j','k','l']  │
└─────────────────────────────────────────────────────────────┘

The long way (equivalent code):
┌─────────────────────────────────────────────────────────────┐
│  result = []                                                │
│  for i in range(97, 109):     # ord('a')=97, ord('l')=108   │
│      result.append(chr(i))    # Convert number to letter    │
│  # result is now ['a', 'b', 'c', ...]                       │
└─────────────────────────────────────────────────────────────┘
```

---

### Exercise 9: Dictionary Practice

**Answer:**
```python
# Create the dictionary
scoring_levels = {
    "loading": 30.5,
    "level1": 31.0,
    "level2": 29.75,
    "level3": 45.25,
    "level4": 78.5
}

# 1. Print height for level3
print(scoring_levels["level3"])  # Output: 45.25

# 2. Add ground entry
scoring_levels["ground"] = 25.0

# 3. Loop through and print all
for level, height in scoring_levels.items():
    print(f"{level}: {height} inches")
```

**Output:**
```
loading: 30.5 inches
level1: 31.0 inches
level2: 29.75 inches
level3: 45.25 inches
level4: 78.5 inches
ground: 25.0 inches
```

**Visual representation:**
```
Dictionary structure:
┌────────────────────────────────────────────────┐
│  scoring_levels = {                            │
│      KEY          VALUE                        │
│      ───          ─────                        │
│      "loading" :  30.5,   ←── First entry      │
│      "level1"  :  31.0,                        │
│      "level2"  :  29.75,                       │
│      "level3"  :  45.25,                       │
│      "level4"  :  78.5,                        │
│      "ground"  :  25.0    ←── Added later      │
│  }                                             │
└────────────────────────────────────────────────┘

Accessing values:
┌────────────────────────────────────────────────┐
│  scoring_levels["level3"]                      │
│                  ─────────                     │
│                  This is the KEY               │
│                       │                        │
│                       ▼                        │
│              Returns: 45.25                    │
│                       ─────                    │
│                  This is the VALUE             │
└────────────────────────────────────────────────┘

.items() returns pairs:
┌────────────────────────────────────────────────┐
│  for level, height in scoring_levels.items():  │
│      ─────  ──────                             │
│        │       │                               │
│        │       └── Gets the value (30.5, etc.) │
│        │                                       │
│        └── Gets the key ("loading", etc.)      │
│                                                │
│  Iteration 1: level="loading", height=30.5     │
│  Iteration 2: level="level1",  height=31.0     │
│  ... and so on                                 │
└────────────────────────────────────────────────┘
```

---

### Exercise 10: Write a Function

**Answer:**
```python
def is_in_range(value, min_val, max_val):
    """
    Check if value is between min_val and max_val (inclusive).

    Args:
        value: The number to check
        min_val: The minimum allowed value
        max_val: The maximum allowed value

    Returns:
        True if value is in range, False otherwise
    """
    return min_val <= value <= max_val

# Test it
print(is_in_range(50, 30, 78))   # True - 50 is between 30 and 78
print(is_in_range(20, 30, 78))   # False - 20 is less than 30
print(is_in_range(30, 30, 78))   # True - 30 equals min (inclusive)
print(is_in_range(78, 30, 78))   # True - 78 equals max (inclusive)
print(is_in_range(100, 30, 78))  # False - 100 is greater than 78
```

**Understanding the function:**
```
Function breakdown:
┌─────────────────────────────────────────────────────────────┐
│  def is_in_range(value, min_val, max_val):                  │
│      ─────────── ────── ──────── ───────                    │
│           │        │       │        │                       │
│           │        │       │        └── Third parameter     │
│           │        │       └── Second parameter             │
│           │        └── First parameter                      │
│           └── Function name                                 │
│                                                             │
│      return min_val <= value <= max_val                     │
│             ───────────────────────────                     │
│             This is a "chained comparison"                  │
│             Python evaluates: (min <= value) AND (value <= max) │
└─────────────────────────────────────────────────────────────┘

Visual test for is_in_range(50, 30, 78):
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│    0    10    20    30    40    50    60    70    78   90   │
│    ├─────┼─────┼─────┼─────┼─────┼─────┼─────┼────┼────┤    │
│                      │           │           │              │
│                      │     ┌─────┘           │              │
│                      │     │                 │              │
│                   min_val value           max_val           │
│                      │     │                 │              │
│                      └─────┼─────────────────┘              │
│                            │                                │
│                      VALID RANGE                            │
│                                                             │
│    Is 30 <= 50?  YES                                        │
│    Is 50 <= 78?  YES                                        │
│    Both true → return True                                  │
└─────────────────────────────────────────────────────────────┘

Alternative implementation (explicit):
┌─────────────────────────────────────────────────────────────┐
│  def is_in_range(value, min_val, max_val):                  │
│      if value < min_val:                                    │
│          return False  # Too low                            │
│      if value > max_val:                                    │
│          return False  # Too high                           │
│      return True       # Just right!                        │
└─────────────────────────────────────────────────────────────┘
```

---

## Advanced Exercises

### Exercise 11: Trace Inheritance

**Answers:**

1. **Robot inherits from:** `commands2.TimedCommandRobot`
   ```python
   class Robot(commands2.TimedCommandRobot):
   ```

2. **Methods Robot overrides:**
   - `robotInit()` - called once when robot starts
   - `robotPeriodic()` - called every loop iteration
   - `autonomousInit()` - called when auto period starts
   - `teleopInit()` - called when teleop period starts
   - `testInit()` - called when test mode starts

3. **`super().__init__()`:** Not explicitly called in our Robot class because `TimedCommandRobot`'s `__init__` doesn't require it for basic usage. The parent class sets itself up through other mechanisms.

**Visual inheritance:**
```
Inheritance hierarchy:
┌─────────────────────────────────────┐
│         WPILib Base Classes         │
│  (written by FRC, we don't see it)  │
└──────────────────┬──────────────────┘
                   │ inherits from
                   ▼
┌─────────────────────────────────────┐
│     commands2.TimedCommandRobot     │
│  ┌───────────────────────────────┐  │
│  │ Provides:                     │  │
│  │  - robotInit()                │  │
│  │  - robotPeriodic()            │  │
│  │  - autonomousInit()           │  │
│  │  - teleopInit()               │  │
│  │  - testInit()                 │  │
│  │  - Command scheduler          │  │
│  └───────────────────────────────┘  │
└──────────────────┬──────────────────┘
                   │ inherits from
                   ▼
┌─────────────────────────────────────┐
│            Our Robot class          │
│  ┌───────────────────────────────┐  │
│  │ Overrides:                    │  │
│  │  - robotInit() ← Add our code │  │
│  │  - autonomousInit()           │  │
│  │  - teleopInit()               │  │
│  │                               │  │
│  │ Gets for free:                │  │
│  │  - Timing loop (20ms cycle)   │  │
│  │  - Period detection           │  │
│  │  - SmartDashboard connection  │  │
│  └───────────────────────────────┘  │
└─────────────────────────────────────┘

When robotInit() is called:
┌─────────────────────────────────────────────────────────────┐
│  1. Robot code starts                                       │
│  2. WPILib creates an instance of our Robot class           │
│  3. Robot.__init__() runs (if we had one)                   │
│  4. robotInit() is called by the framework                  │
│                                                             │
│  Our robotInit():                                           │
│  ┌─────────────────────────────────────────────────────┐    │
│  │ def robotInit(self):                                │    │
│  │     self.container = RobotContainer()  # Set up robot│   │
│  │     self.autonomous_command = None     # No auto yet │   │
│  └─────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────┘
```

---

### Exercise 12: Understand self

**Answers from `src/subsystems/elevator.py`:**

1. **Three attributes using `self.`:**
   ```python
   self.left_motor     # Reference to left motor controller
   self.right_motor    # Reference to right motor controller
   self.encoder        # Reference to the encoder
   self.target_height  # The target height to move to
   self.pid_controller # The PID controller
   ```

2. **Method calling another method via `self.`:**
   ```python
   def at_goal_height(self) -> bool:
       # This method calls carriage_height() via self
       return abs(self.carriage_height() - self.target_height) < ElevatorConstants.TOLERANCE
   ```

3. **Why `self` is needed:**
   - `self` refers to the specific instance of the class
   - Without it, Python wouldn't know which elevator's data to access
   - Each Elevator object has its own separate `height`, `target`, etc.

**Visual explanation of self:**
```
Why self matters - multiple instances:
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│  If we had two elevators (we don't, but imagine):           │
│                                                             │
│  elevator1 = Elevator()        elevator2 = Elevator()       │
│  ┌──────────────────┐          ┌──────────────────┐         │
│  │ self.height = 30 │          │ self.height = 50 │         │
│  │ self.target = 45 │          │ self.target = 70 │         │
│  │ self.motor = ... │          │ self.motor = ... │         │
│  └──────────────────┘          └──────────────────┘         │
│                                                             │
│  elevator1.carriage_height()   elevator2.carriage_height()  │
│       │                              │                      │
│       │                              │                      │
│       ▼                              ▼                      │
│  Returns 30                    Returns 50                   │
│  (elevator1's height)          (elevator2's height)         │
│                                                             │
└─────────────────────────────────────────────────────────────┘

Inside the method:
┌─────────────────────────────────────────────────────────────┐
│  def carriage_height(self) -> float:                        │
│      return self.encoder.getPosition()                      │
│             ────                                            │
│              │                                              │
│              └── "self" is automatically set to the         │
│                  object the method was called on            │
│                                                             │
│  When you call:  elevator1.carriage_height()                │
│  Python does:    Elevator.carriage_height(elevator1)        │
│                                            ─────────        │
│                                         self = elevator1    │
└─────────────────────────────────────────────────────────────┘

Common mistake - forgetting self:
┌─────────────────────────────────────────────────────────────┐
│  WRONG:                          RIGHT:                     │
│  def set_height(height):         def set_height(self, height): │
│      target = height                 self.target = height   │
│                                                             │
│  The wrong version creates a     The right version sets     │
│  local variable that disappears  the object's attribute     │
│  when the function ends!         that persists              │
└─────────────────────────────────────────────────────────────┘
```

---

### Exercise 13: Command Chaining

**Example from `src/container.py`:**
```python
auto = (
    self.swerve.follow_path("Start to Reef")
    .andThen(commands2.WaitCommand(0.25))
    .andThen(self.score_coral())
    .withTimeout(2)
)
```

**Breakdown:**

1. **The chain:** `follow_path() → andThen(wait) → andThen(score) → withTimeout(2)`

2. **What each part does:**
   - `follow_path("Start to Reef")` - Drive the swerve along a pre-planned path
   - `.andThen(WaitCommand(0.25))` - After path completes, wait 0.25 seconds
   - `.andThen(self.score_coral())` - After wait, run the scoring command
   - `.withTimeout(2)` - Cancel everything if it takes longer than 2 seconds

3. **Execution order:** Sequential from left to right (follow path → wait → score)

**Visual command flow:**
```
Command chain execution:
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│  Step 1: follow_path("Start to Reef")                       │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  Robot drives along the path...                     │    │
│  │  [═══════════════════════════════>]                 │    │
│  │                                    ↓ path complete  │    │
│  └─────────────────────────────────────────────────────┘    │
│                                                             │
│  Step 2: WaitCommand(0.25)                                  │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  Wait 0.25 seconds...                               │    │
│  │  [====]                                             │    │
│  │       ↓ time elapsed                                │    │
│  └─────────────────────────────────────────────────────┘    │
│                                                             │
│  Step 3: score_coral()                                      │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  Run scoring sequence...                            │    │
│  │  [═══════════>]                                     │    │
│  │              ↓ scoring complete                     │    │
│  └─────────────────────────────────────────────────────┘    │
│                                                             │
│  The withTimeout(2) applies to THE WHOLE THING:             │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  |-------- 2 second timeout window --------|        │    │
│  │  If Step 1 + Step 2 + Step 3 takes > 2 seconds,     │    │
│  │  everything gets cancelled!                         │    │
│  └─────────────────────────────────────────────────────┘    │
│                                                             │
└─────────────────────────────────────────────────────────────┘

How chaining works internally:
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│  command1.andThen(command2)                                 │
│                                                             │
│  Creates a NEW SequentialCommandGroup containing:           │
│  ┌──────────────────────────────────────────────┐           │
│  │  SequentialCommandGroup                      │           │
│  │  ┌──────────┐    ┌──────────┐                │           │
│  │  │ command1 │ →  │ command2 │                │           │
│  │  └──────────┘    └──────────┘                │           │
│  └──────────────────────────────────────────────┘           │
│                                                             │
│  Each .andThen() or .withTimeout() returns a NEW command,   │
│  which is why you can chain them!                           │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

### Exercise 14: Button Binding

**Answers from `src/container.py`:**

1. **Button using `.onTrue()`:**
   ```python
   self.driver_joystick.start_button.onTrue(
       commands2.InstantCommand(lambda: self.swerve.reset_gyro(Rotation2d.fromDegrees(180)))
   )
   ```

2. **Button using `.whileTrue()`:**
   ```python
   self.operator_joystick.right_trigger.whileTrue(
       self.superstructure.LoadingPositionCommand(...)
   )
   ```

3. **Difference:**
   - **onTrue**: Runs once when button is pressed, continues even if released
   - **whileTrue**: Runs while held, stops when released

**Visual comparison:**
```
onTrue vs whileTrue:
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│  Button state:      ___████████████___                      │
│                     ↑              ↑                        │
│                  pressed        released                    │
│                                                             │
│  ─────────────────────────────────────────────────────────  │
│                                                             │
│  onTrue behavior:                                           │
│  Command:           ___████████████████████████___          │
│                        ↑                                    │
│                     starts here, keeps running until        │
│                     command finishes on its own             │
│                                                             │
│  Use for: Instant actions, toggles, commands that          │
│           should complete their task                        │
│                                                             │
│  ─────────────────────────────────────────────────────────  │
│                                                             │
│  whileTrue behavior:                                        │
│  Command:           ___████████████___                      │
│                        ↑           ↑                        │
│                     starts      stops when                  │
│                                 button released             │
│                                                             │
│  Use for: Continuous actions like driving, holding,         │
│           or actions that should stop immediately           │
│                                                             │
└─────────────────────────────────────────────────────────────┘

Other trigger methods:
┌─────────────────────────────────────────────────────────────┐
│  .onTrue()     - When button pressed (rising edge)          │
│  .onFalse()    - When button released (falling edge)        │
│  .whileTrue()  - While button held                          │
│  .whileFalse() - While button NOT held                      │
│  .toggleOnTrue() - Toggle command on/off with each press    │
└─────────────────────────────────────────────────────────────┘
```

---

### Exercise 15: Modify a Constant

**Answers:**

1. **Where scoring heights are defined:**
   - In `src/constants.py`, look for `ScoringConstants` class

2. **To change Level 2 from 29.75 to 30.0:**
   ```python
   # Before:
   LEVEL_2_HEIGHT = 29.75

   # After:
   LEVEL_2_HEIGHT = 30.0
   ```

3. **Files potentially affected:**
   - Any file that imports and uses `ScoringConstants.LEVEL_2_HEIGHT`
   - `src/container.py` (button bindings)
   - `src/subsystems/superstructure.py` (if it references this constant)
   - No code changes needed in those files - they reference the constant by name!

**Why constants are powerful:**
```
Without constants (BAD):
┌─────────────────────────────────────────────────────────────┐
│  # elevator.py                                              │
│  if height > 29.75:  # What is this number??                │
│                                                             │
│  # container.py                                             │
│  elevator.set_height(29.75)  # Magic number!                │
│                                                             │
│  # superstructure.py                                        │
│  target = 29.75  # Same magic number...                     │
│                                                             │
│  To change Level 2 height, you'd have to:                   │
│  1. Search entire codebase for "29.75"                      │
│  2. Figure out which ones are Level 2 (not all 29.75s are!) │
│  3. Change each one individually                            │
│  4. Hope you didn't miss any!                               │
└─────────────────────────────────────────────────────────────┘

With constants (GOOD):
┌─────────────────────────────────────────────────────────────┐
│  # constants.py                                             │
│  class ScoringConstants:                                    │
│      LEVEL_2_HEIGHT = 29.75  ← Change ONLY here!            │
│                                                             │
│  # elevator.py                                              │
│  if height > ScoringConstants.LEVEL_2_HEIGHT:  # Clear!     │
│                                                             │
│  # container.py                                             │
│  elevator.set_height(ScoringConstants.LEVEL_2_HEIGHT)       │
│                                                             │
│  To change Level 2 height:                                  │
│  1. Change ONE line in constants.py                         │
│  2. Done! All references automatically use new value        │
└─────────────────────────────────────────────────────────────┘
```

---

### Exercise 16: Add a Button Binding

**Answers:**

1. **File to modify:** `src/container.py`

2. **Method to add code to:** `configure_button_bindings()`

3. **The code to add:**
   ```python
   # In configure_button_bindings() method:
   self.operator_joystick.y_button.onTrue(
       commands2.PrintCommand("Y pressed!")
   )
   ```

4. **Complete picture:**
   ```python
   def configure_button_bindings(self):
       # ... existing bindings ...

       # Add this new binding:
       self.operator_joystick.y_button.onTrue(
           commands2.PrintCommand("Y pressed!")
       )
   ```

**Understanding the binding:**
```
Button binding anatomy:
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│  self.operator_joystick.y_button.onTrue(                    │
│  ──────────────────────  ────────  ──────                   │
│           │                │         │                      │
│           │                │         └── When to trigger    │
│           │                │             (on press)         │
│           │                │                                │
│           │                └── Which button (Y button)      │
│           │                                                 │
│           └── Which controller (operator's Xbox)            │
│                                                             │
│      commands2.PrintCommand("Y pressed!")                   │
│      ─────────────────────────────────────                  │
│                      │                                      │
│                      └── What to do (print message)         │
│                                                             │
└─────────────────────────────────────────────────────────────┘

Flow when Y is pressed:
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│  1. Operator presses Y button                               │
│     ┌─────────────┐                                         │
│     │  Controller │                                         │
│     │   [Y]       │ ← Press!                                │
│     └─────────────┘                                         │
│            │                                                │
│            ▼                                                │
│  2. WPILib detects button state change (False → True)       │
│            │                                                │
│            ▼                                                │
│  3. Trigger's onTrue() condition is met                     │
│            │                                                │
│            ▼                                                │
│  4. PrintCommand is scheduled                               │
│            │                                                │
│            ▼                                                │
│  5. PrintCommand runs, prints "Y pressed!" to console       │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

### Exercise 17: Create a Simple Subsystem

**Answer - LED Subsystem Design:**

```python
import commands2
from wpilib import AddressableLED

class LED(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        # Hardware setup
        self.led_strip = AddressableLED(0)  # PWM port 0
        self.led_strip.setLength(60)        # 60 LEDs
        self.led_strip.start()

        # State
        self.color = "off"

    # Getter method
    def get_color(self) -> str:
        """Returns the current color name."""
        return self.color

    # Setter method
    def set_color(self, color: str) -> None:
        """Set the LED strip to a color."""
        self.color = color
        # Would actually set LED colors here based on color name

    def periodic(self) -> None:
        """Update LEDs (called every 20ms)."""
        pass

    # Inner command class
    class SetColorCommand(commands2.Command):
        """Command to change LED color."""

        def __init__(self, led_subsystem, color: str):
            super().__init__()
            self.led = led_subsystem
            self.color = color
            self.addRequirements(led_subsystem)

        def initialize(self):
            self.led.set_color(self.color)

        def isFinished(self) -> bool:
            return True  # Instant command
```

**Design diagram:**
```
LED Subsystem Structure:
┌─────────────────────────────────────────────────────────────┐
│                       LED (Subsystem)                       │
├─────────────────────────────────────────────────────────────┤
│  ATTRIBUTES:                                                │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  self.led_strip : AddressableLED                    │    │
│  │  self.color : str = "off"                           │    │
│  └─────────────────────────────────────────────────────┘    │
├─────────────────────────────────────────────────────────────┤
│  METHODS:                                                   │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  get_color() → str          # Returns current color │    │
│  │  set_color(color: str)      # Changes the color     │    │
│  │  periodic()                  # Called every 20ms    │    │
│  └─────────────────────────────────────────────────────┘    │
├─────────────────────────────────────────────────────────────┤
│  INNER CLASSES:                                             │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  SetColorCommand(Command)                           │    │
│  │  ├── __init__(led, color)   # Store references      │    │
│  │  ├── initialize()           # Set the color         │    │
│  │  └── isFinished() → True    # Instant command       │    │
│  └─────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────┘

Usage:
┌─────────────────────────────────────────────────────────────┐
│  # In container.py                                          │
│  self.led = LED()                                           │
│                                                             │
│  # Set color directly                                       │
│  self.led.set_color("green")                                │
│                                                             │
│  # Or use a command (for button bindings)                   │
│  button.onTrue(LED.SetColorCommand(self.led, "green"))      │
└─────────────────────────────────────────────────────────────┘
```

---

### Exercise 18: Understand Triggers

**Example from `src/container.py`:**

```python
Trigger(
    lambda: self.claw.has_piece() and wpilib.DriverStation.isTeleop()
).whileTrue(
    self.led.set_pattern("has_piece")
)
```

**Answers:**

1. **Condition checked:** `self.claw.has_piece() and wpilib.DriverStation.isTeleop()`
   - The claw must have a game piece AND we must be in teleop mode

2. **Command run:** `self.led.set_pattern("has_piece")` - Changes LED pattern to indicate we have a piece

3. **When it runs:** `whileTrue` - runs continuously while the condition is true, stops when it becomes false

**Understanding Triggers:**
```
Trigger breakdown:
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│  Trigger(lambda: self.claw.has_piece() and DriverStation.isTeleop()) │
│  ───────────────────────────────────────────────────────────│
│         │                                                   │
│         └── Lambda function that returns True/False         │
│             This is evaluated every robot loop (20ms)       │
│                                                             │
│  .whileTrue(command)                                        │
│  ───────────────────                                        │
│         │                                                   │
│         └── What to do while the lambda returns True        │
│                                                             │
└─────────────────────────────────────────────────────────────┘

Timeline example:
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│  Time:    0s    1s    2s    3s    4s    5s    6s            │
│           │     │     │     │     │     │     │             │
│  has_piece: F    F    T     T     T     F     F             │
│  isTeleop:  T    T    T     T     T     T     T             │
│           │     │     │     │     │     │     │             │
│  Trigger:   F    F    T     T     T     F     F             │
│                       │─────────────│                       │
│                       ▼             ▼                       │
│  Command:           START ════════ STOP                     │
│                    (LED shows "has piece" pattern)          │
│                                                             │
│  The LED only shows the pattern while we have the piece!    │
│                                                             │
└─────────────────────────────────────────────────────────────┘

Custom Trigger vs Button:
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│  Button trigger (hardware):                                 │
│  self.driver.a_button.onTrue(command)                       │
│       │                                                     │
│       └── a_button IS already a Trigger!                    │
│           (created from controller input)                   │
│                                                             │
│  Custom trigger (software condition):                       │
│  Trigger(lambda: some_condition()).whileTrue(command)       │
│       │                                                     │
│       └── We CREATE a Trigger from any boolean condition    │
│                                                             │
│  Both work the same way - the difference is what            │
│  determines True/False (button vs code logic)               │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

### Exercise 19: Type Hints

**Answers from `src/vision.py`:**

1. **Function returning Optional:**
   ```python
   def get_pose_estimation(self) -> Optional[tuple[Pose2d, float]]:
   ```

2. **What Optional means:**
   - The function might return `tuple[Pose2d, float]` (a pose and timestamp)
   - OR it might return `None` (no valid pose found)
   - `Optional[X]` is the same as `X | None`

3. **Why it might return None:**
   - No AprilTags visible to the camera
   - Camera isn't connected
   - Pose estimation failed
   - Data was too old/unreliable

**Understanding Optional:**
```
Optional explained:
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│  def get_pose_estimation(self) -> Optional[tuple[Pose2d, float]]: │
│                                   ────────────────────────  │
│                                           │                 │
│                                           │                 │
│                                           ▼                 │
│                                   Can return either:        │
│                                   ┌─────────────────────┐   │
│                                   │ tuple[Pose2d, float]│   │
│                                   │  (pose, timestamp)  │   │
│                                   └─────────────────────┘   │
│                                          OR                 │
│                                   ┌─────────────────────┐   │
│                                   │        None         │   │
│                                   │   (no valid data)   │   │
│                                   └─────────────────────┘   │
│                                                             │
└─────────────────────────────────────────────────────────────┘

Why Optional matters:
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│  WRONG - doesn't check for None:                            │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  result = vision.get_pose_estimation()              │    │
│  │  pose = result[0]  # CRASH if result is None!       │    │
│  └─────────────────────────────────────────────────────┘    │
│                                                             │
│  RIGHT - handles None case:                                 │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  result = vision.get_pose_estimation()              │    │
│  │  if result is not None:                             │    │
│  │      pose, timestamp = result                       │    │
│  │      # Use the pose...                              │    │
│  │  else:                                              │    │
│  │      # No valid pose available                      │    │
│  └─────────────────────────────────────────────────────┘    │
│                                                             │
└─────────────────────────────────────────────────────────────┘

Vision scenarios:
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│  Scenario 1: AprilTag visible                               │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  Camera sees tag #3                                 │    │
│  │           ↓                                         │    │
│  │  Calculate robot position from tag                  │    │
│  │           ↓                                         │    │
│  │  Return (Pose2d(x, y, rotation), timestamp)         │    │
│  └─────────────────────────────────────────────────────┘    │
│                                                             │
│  Scenario 2: No AprilTag visible                            │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  Camera sees... nothing useful                      │    │
│  │           ↓                                         │    │
│  │  Can't calculate position                           │    │
│  │           ↓                                         │    │
│  │  Return None                                        │    │
│  └─────────────────────────────────────────────────────┘    │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

### Exercise 20: Full Trace

**Tracing the A button press (this is a detailed walkthrough):**

Starting point: Driver presses A button during teleop.

**Step 1: Find the binding in `src/container.py`:**
```python
self.operator_joystick.a_button.onTrue(
    self.superstructure.SetLevelCommand(self, ScoringConstants.LEVEL_1_HEIGHT, ScoringConstants.LEVEL_1_ANGLE)
)
```

**Step 2: The command triggered:**
`SetLevelCommand` from the superstructure, targeting Level 1 height and angle

**Step 3: What subsystems are used:**
- Elevator (to move to height)
- Coral Arm (to move to angle)
- Possibly Claw (to hold piece)

**Step 4: Methods called:**
- `elevator.set_height(LEVEL_1_HEIGHT)`
- `coral_arm.set_angle(LEVEL_1_ANGLE)`

**Full trace diagram:**
```
Complete button press trace:
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│  1. PHYSICAL INPUT                                          │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  Driver presses A button on Xbox controller         │    │
│  │  ┌───────────────┐                                  │    │
│  │  │  Xbox         │                                  │    │
│  │  │  ┌───┐        │                                  │    │
│  │  │  │ A │ ← PRESS│                                  │    │
│  │  │  └───┘        │                                  │    │
│  │  └───────────────┘                                  │    │
│  └─────────────────────────────────────────────────────┘    │
│                         │                                   │
│                         ▼                                   │
│  2. WPILIB DETECTION                                        │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  WPILib reads controller state via USB              │    │
│  │  operator_joystick.a_button goes from False → True  │    │
│  │  This is the "rising edge" that triggers onTrue     │    │
│  └─────────────────────────────────────────────────────┘    │
│                         │                                   │
│                         ▼                                   │
│  3. TRIGGER FIRES                                           │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  a_button.onTrue() condition is met                 │    │
│  │  The associated command gets scheduled              │    │
│  └─────────────────────────────────────────────────────┘    │
│                         │                                   │
│                         ▼                                   │
│  4. COMMAND SCHEDULED                                       │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  SetLevelCommand(LEVEL_1_HEIGHT, LEVEL_1_ANGLE)     │    │
│  │  is added to the command scheduler                  │    │
│  │                                                     │    │
│  │  Command requirements: [elevator, coral_arm]        │    │
│  │  Any commands currently using these subsystems      │    │
│  │  get interrupted!                                   │    │
│  └─────────────────────────────────────────────────────┘    │
│                         │                                   │
│                         ▼                                   │
│  5. COMMAND EXECUTES                                        │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  SetLevelCommand.initialize():                      │    │
│  │    - Sets elevator target to 31.0 inches            │    │
│  │    - Sets arm target to -35 degrees                 │    │
│  │                                                     │    │
│  │  SetLevelCommand.execute(): (every 20ms)            │    │
│  │    - Elevator PID moves toward 31.0"                │    │
│  │    - Arm PID moves toward -35°                      │    │
│  │                                                     │    │
│  │  SetLevelCommand.isFinished():                      │    │
│  │    - Returns True when both are at target           │    │
│  └─────────────────────────────────────────────────────┘    │
│                         │                                   │
│                         ▼                                   │
│  6. SUBSYSTEM METHODS CALLED                                │
│  ┌─────────────────────────────────────────────────────┐    │
│  │                                                     │    │
│  │  Elevator:                    Coral Arm:            │    │
│  │  ┌──────────────────────┐    ┌──────────────────┐   │    │
│  │  │ set_height(31.0)     │    │ set_angle(-35)   │   │    │
│  │  │   │                  │    │   │              │   │    │
│  │  │   ▼                  │    │   ▼              │   │    │
│  │  │ PID controller       │    │ PID controller   │   │    │
│  │  │ calculates output    │    │ calculates output│   │    │
│  │  │   │                  │    │   │              │   │    │
│  │  │   ▼                  │    │   ▼              │   │    │
│  │  │ Motors spin to       │    │ Motor spins to   │   │    │
│  │  │ move carriage        │    │ rotate arm       │   │    │
│  │  └──────────────────────┘    └──────────────────┘   │    │
│  │                                                     │    │
│  └─────────────────────────────────────────────────────┘    │
│                         │                                   │
│                         ▼                                   │
│  7. COMMAND COMPLETES                                       │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  isFinished() returns True                          │    │
│  │  Command is removed from scheduler                  │    │
│  │  Robot is now at Level 1 scoring position!          │    │
│  └─────────────────────────────────────────────────────┘    │
│                                                             │
└─────────────────────────────────────────────────────────────┘

Physical result:
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│  BEFORE (Loading position):    AFTER (Level 1):             │
│                                                             │
│       ┌───┐                         ┌───┐                   │
│       │ARM│ (0°)                    │ARM│ (-35°)            │
│       └─┬─┘                         └─┬─┘                   │
│         │                             │\                    │
│    ┌────┴────┐                   ┌────┴────┐                │
│    │ELEVATOR │ (30.5")           │ELEVATOR │ (31")          │
│    │         │                   │         │                │
│    └─────────┘                   └─────────┘                │
│    ════════════                  ════════════               │
│       ROBOT                         ROBOT                   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## Congratulations!

If you've completed these exercises, you have a solid understanding of Python and how our robot code works. You're ready to:

1. Read and understand any file in the codebase
2. Make small modifications safely
3. Add new button bindings
4. Modify constants for tuning
5. Understand error messages and fix simple bugs

**Next steps:**
- Try modifying actual code in simulation
- Ask a mentor to review your changes
- Work on adding a new feature with guidance

**Remember:** Every expert was once a beginner. Keep practicing!
