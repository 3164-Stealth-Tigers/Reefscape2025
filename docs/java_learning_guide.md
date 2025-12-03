# Java Learning Guide for Team 3164

A progressive guide to learn Java by working with our robot code. Each section builds on the previous one.

---

# Getting Started from Scratch

This section will help you set up everything you need to start programming in Java on your Windows PC. Follow each step carefully!

---

## Step 1: Install Java JDK (Java Development Kit)

The JDK is the software that lets you write and run Java programs. We'll install **JDK 17** which is the recommended version for FRC.

### 1.1 Download the JDK

1. Open your web browser (Chrome, Edge, Firefox, etc.)
2. Go to: **https://adoptium.net/**
3. You should see a big download button. Make sure it says:
   - **Temurin 17** (or the latest LTS version)
   - **Windows x64**
   - **.msi** (installer)
4. Click the download button

If the website doesn't automatically detect Windows, click on "Other platforms" and select:
- Operating System: **Windows**
- Architecture: **x64**
- Package Type: **JDK**
- Version: **17 - LTS**

### 1.2 Install the JDK

1. Find the downloaded file (usually in your Downloads folder)
   - It will be named something like `OpenJDK17U-jdk_x64_windows_hotspot_17.0.x.msi`
2. Double-click the file to run the installer
3. If Windows asks "Do you want to allow this app to make changes?" click **Yes**
4. The installer wizard will open. Click **Next**
5. On the "Custom Setup" screen, make sure these options are checked:
   - ✅ Add to PATH
   - ✅ Set JAVA_HOME variable
   - ✅ JavaSoft (Oracle) registry keys
6. Click **Next**, then click **Install**
7. Wait for the installation to complete
8. Click **Finish**

### 1.3 Verify Java is Installed

1. Press the **Windows key** on your keyboard
2. Type **cmd** and press Enter (this opens Command Prompt)
3. In the black window that appears, type:
   ```
   java -version
   ```
4. Press Enter
5. You should see something like:
   ```
   openjdk version "17.0.9" 2023-10-17
   OpenJDK Runtime Environment Temurin-17.0.9+9 (build 17.0.9+9)
   OpenJDK 64-Bit Server VM Temurin-17.0.9+9 (build 17.0.9+9, mixed mode, sharing)
   ```
6. Now type:
   ```
   javac -version
   ```
7. Press Enter. You should see:
   ```
   javac 17.0.9
   ```

**If you see an error** like "'java' is not recognized":
- Restart your computer and try again
- If it still doesn't work, the PATH wasn't set correctly. Ask a mentor for help.

---

## Step 2: Install IntelliJ IDEA

IntelliJ IDEA is an IDE (Integrated Development Environment) - a program that helps you write code. It provides helpful features like code completion, error highlighting, and easy project management.

### 2.1 Download IntelliJ IDEA Community Edition

1. Go to: **https://www.jetbrains.com/idea/download/**
2. Scroll down to find **IntelliJ IDEA Community Edition** (the FREE version)
   - **Important:** Don't download the Ultimate version - it's not free!
   - The Community edition is on the RIGHT side of the page
3. Click the **Download** button under "Community"
4. Wait for the download to complete (it's about 500-700 MB)

### 2.2 Install IntelliJ IDEA

1. Find the downloaded file (something like `ideaIC-2024.x.x.exe`)
2. Double-click to run the installer
3. If Windows asks for permission, click **Yes**
4. Click **Next** on the welcome screen
5. Choose the installation location (the default is fine) and click **Next**
6. On the "Installation Options" screen, check these boxes:
   - ✅ 64-bit launcher (creates a desktop shortcut)
   - ✅ Add "Open Folder as Project" (helpful for opening projects)
   - ✅ .java (associates Java files with IntelliJ)
   - ✅ Add launchers dir to the PATH
7. Click **Next**
8. Click **Install**
9. Wait for the installation to complete
10. Check "Run IntelliJ IDEA Community Edition" and click **Finish**

### 2.3 First-Time Setup

When IntelliJ opens for the first time:

1. You'll see "Import IntelliJ IDEA Settings"
   - Select **"Do not import settings"** and click **OK**
2. Accept the User Agreement and click **Continue**
3. Choose whether to send anonymous usage statistics (your choice)
4. You'll see the Welcome screen - you're ready to create your first project!

---

## Step 3: Create Your First Java Program (Hello World!)

Now let's write your first Java program that prints "Hello, World!" to the screen.

### 3.1 Create a New Project

1. On the IntelliJ Welcome screen, click **"New Project"**
2. On the left side, make sure **"Java"** is selected
3. Configure your project:
   - **Name:** `HelloWorld`
   - **Location:** Leave as default, or choose a folder you'll remember (like Documents)
   - **JDK:** Click the dropdown and select **17** (or "temurin-17")
     - If you don't see it, click "Download JDK" and select version 17
   - Leave other options as default
4. Click **Create**
5. If asked about creating a sample application, click **Yes** or just proceed

### 3.2 Create the Main Class

1. In the left panel, you'll see your project structure:
   ```
   HelloWorld
   └── src
   ```
2. Right-click on the **src** folder
3. Select **New** → **Java Class**
4. Type `Main` for the name and press Enter
5. IntelliJ creates a new file with this code:
   ```java
   public class Main {
   }
   ```

### 3.3 Write the Hello World Code

1. Click inside the curly braces `{ }` after `public class Main`
2. Add a new line and type this code exactly:

```java
public class Main {
    public static void main(String[] args) {
        System.out.println("Hello, World!");
    }
}
```

**Let's break down what each part means:**

| Code | What It Means |
|------|---------------|
| `public class Main` | Creates a class named "Main" (the container for our code) |
| `public static void main(String[] args)` | The main method - where the program starts running |
| `System.out.println("Hello, World!");` | Prints text to the screen |
| `{ }` | Curly braces group code together (like Python's indentation) |
| `;` | Semicolon ends each statement (required in Java!) |

### 3.4 Run Your Program

**Method 1: Using the Green Play Button**
1. Look at the left margin next to line 1 or line 2
2. You'll see a small green **▶** (play) icon
3. Click on it
4. Select **"Run 'Main.main()'"**

**Method 2: Using the Keyboard**
1. Press **Ctrl + Shift + F10** (or **Shift + F10** if main was already run)

**Method 3: Using the Menu**
1. Click **Run** in the top menu
2. Click **Run 'Main'**

### 3.5 See the Output

After running, look at the bottom of the IntelliJ window. A "Run" panel will appear showing:
```
Hello, World!

Process finished with exit code 0
```

**Congratulations!** 🎉 You've written and run your first Java program!

---

## Step 4: Try Some Experiments

Now let's modify the program to learn more. Try each of these:

### Experiment 1: Change the Message

Change the text inside the quotes and run again:
```java
System.out.println("Go Stealth Tigers!");
```

### Experiment 2: Print Multiple Lines

Add more print statements:
```java
public class Main {
    public static void main(String[] args) {
        System.out.println("Hello, World!");
        System.out.println("My name is [Your Name]");
        System.out.println("I'm learning Java!");
    }
}
```

### Experiment 3: Use Variables

Store values in variables and print them:
```java
public class Main {
    public static void main(String[] args) {
        String teamName = "Stealth Tigers";
        int teamNumber = 3164;

        System.out.println("Team: " + teamName);
        System.out.println("Number: " + teamNumber);
    }
}
```

### Experiment 4: Do Some Math

Java can do calculations:
```java
public class Main {
    public static void main(String[] args) {
        int a = 10;
        int b = 5;

        System.out.println("a + b = " + (a + b));
        System.out.println("a - b = " + (a - b));
        System.out.println("a * b = " + (a * b));
        System.out.println("a / b = " + (a / b));
    }
}
```

### Experiment 5: Ask for Input

Get input from the user:
```java
import java.util.Scanner;

public class Main {
    public static void main(String[] args) {
        Scanner scanner = new Scanner(System.in);

        System.out.print("What is your name? ");
        String name = scanner.nextLine();

        System.out.println("Hello, " + name + "!");

        scanner.close();
    }
}
```

---

## Common Errors and How to Fix Them

### Error: "Cannot resolve symbol"
- **Cause:** You misspelled something or forgot to import a class
- **Fix:** Check your spelling carefully. Java is case-sensitive!
  - `String` is correct, `string` is wrong
  - `System` is correct, `system` is wrong

### Error: "';' expected"
- **Cause:** You forgot a semicolon at the end of a line
- **Fix:** Add `;` at the end of the statement

### Error: "class, interface, or enum expected"
- **Cause:** Code is outside the class braces
- **Fix:** Make sure all your code is inside `public class Main { }`

### Error: "'java' is not recognized"
- **Cause:** Java wasn't added to your PATH during installation
- **Fix:** Reinstall Java and make sure to check "Add to PATH"

### Error: Red underlines everywhere
- **Cause:** Usually a missing brace `{` or `}`
- **Fix:** Count your opening and closing braces - they must match!

### The program runs but nothing happens
- **Cause:** You might not have the `main` method, or it's spelled wrong
- **Fix:** Make sure you have exactly: `public static void main(String[] args)`

---

## Helpful IntelliJ Shortcuts

| Shortcut | What It Does |
|----------|-------------|
| `Ctrl + Space` | Show code suggestions (autocomplete) |
| `Ctrl + Shift + F10` | Run current file |
| `Shift + F10` | Run last configuration |
| `Ctrl + /` | Comment/uncomment a line |
| `Ctrl + D` | Duplicate current line |
| `Ctrl + Z` | Undo |
| `Ctrl + Shift + Z` | Redo |
| `Ctrl + S` | Save file |
| `Alt + Enter` | Show quick fixes for errors |
| `Ctrl + Alt + L` | Format code nicely |
| `sout` + Tab | Quickly type `System.out.println()` |
| `psvm` + Tab | Quickly type `public static void main(String[] args)` |

---

## IntelliJ Tips for Beginners

### Live Templates (Shortcuts for Common Code)

Instead of typing everything out, IntelliJ has shortcuts:

1. Type `sout` and press **Tab** → expands to `System.out.println();`
2. Type `psvm` and press **Tab** → expands to `public static void main(String[] args) { }`
3. Type `fori` and press **Tab** → creates a for loop template

### Auto-Import

If you use a class that needs an import (like `Scanner`):
1. Type the class name
2. It will be underlined red
3. Press **Alt + Enter**
4. Select "Import class"

### Code Completion

1. Start typing anything
2. Press **Ctrl + Space** to see suggestions
3. Use arrow keys to select and press **Enter**

---

## What's Next?

Now that you have Java and IntelliJ set up, continue with the rest of this guide to learn:

1. **Variables and Types** - Storing different kinds of data
2. **If Statements** - Making decisions in code
3. **Loops** - Repeating actions
4. **Methods** - Creating reusable code blocks
5. **Classes** - Organizing code into objects
6. **FRC Concepts** - Subsystems, Commands, and robot programming!

---

## How to Use This Guide

1. Read each section in order
2. Find the example in our actual code (file paths provided in `src_java/`)
3. Try modifying small things to see what happens
4. Complete the practice exercises at the end
5. Move to the next section when you're comfortable

**Pro tip:** Open the referenced files side-by-side with this guide!

---

# Part 1: The Basics

## 1.1 Variables - Storing Information

Variables are names that hold values. Java is a **statically typed** language, meaning you must declare the type of each variable.

**Why variables matter:** Instead of typing `78.5` everywhere for the max elevator height, we store it once in a variable called `MAX_HEIGHT`. If we need to change it, we only change it in one place!

```java
// Numbers - Java has different types for different sizes
int motorId = 9;                    // Integer (whole number) - 32 bits
long bigNumber = 9999999999L;       // Long integer - 64 bits (note the L)
double maxHeight = 78.5;            // Double precision decimal - 64 bits
float smallDecimal = 3.14f;         // Single precision decimal (note the f)

// Text (strings) - always in double quotes
String cameraName = "front_camera"; // Must use double quotes
String teamName = "Stealth Tigers";
String emptyString = "";            // Empty string is valid

// Single characters use single quotes
char letter = 'A';                  // Single character

// True/False (booleans) - only two possible values
boolean isEnabled = true;           // Note: lowercase true
boolean useAuto = false;            // Note: lowercase false

// Declaring without initializing
int speed;                          // Declared but not initialized
speed = 50;                         // Initialize later

// Constants use 'final' keyword (value cannot change)
final double MAX_HEIGHT = 78.5;
final int MOTOR_ID = 9;
// MAX_HEIGHT = 80.0;              // ERROR! Cannot change a final variable
```

**Naming conventions:**
- Variables use `camelCase` (first word lowercase, capitalize each subsequent word)
- Constants use `UPPER_SNAKE_CASE`
- Class names use `PascalCase`

```java
// Good variable names
double motorSpeed = 0.5;
double elevatorHeight = 45.0;
boolean isAtTarget = true;

// Constants should be UPPER_SNAKE_CASE
final int LEFT_MOTOR_ID = 9;
final double MAX_VELOCITY = 4.2;
```

**In our code:** Look at `src_java/frc/robot/Constants.java` for examples of constant declarations.

---

## 1.2 Basic Math

Java can do all the math you need for robotics calculations.

```java
// Basic arithmetic
int total = 5 + 3;           // Addition: 8
int difference = 10 - 4;     // Subtraction: 6
int product = 3 * 4;         // Multiplication: 12
double quotient = 20.0 / 4;  // Division: 5.0
int integerDiv = 20 / 4;     // Integer division: 5 (no decimal!)
int remainder = 17 % 5;      // Modulo (remainder): 2

// IMPORTANT: Integer division truncates!
int result = 7 / 2;          // Result is 3, not 3.5!
double result2 = 7.0 / 2;    // Result is 3.5

// Order of operations (PEMDAS applies!)
int calc = 2 + 3 * 4;        // 14, not 20 (multiplication first)
int calc2 = (2 + 3) * 4;     // 20 (parentheses first)

// Compound assignment (shortcuts)
int speed = 10;
speed = speed + 5;           // Long way: speed is now 15
speed += 5;                  // Short way: speed is now 20
speed -= 3;                  // speed is now 17
speed *= 2;                  // speed is now 34
speed /= 2;                  // speed is now 17

// Increment and decrement
int count = 0;
count++;                     // count is now 1 (same as count = count + 1)
count--;                     // count is now 0 (same as count = count - 1)

// Math class functions
double absolute = Math.abs(-5);           // Absolute value: 5.0
double smallest = Math.min(3, 7);         // Minimum: 3.0
double largest = Math.max(3, 7);          // Maximum: 7.0
double rounded = Math.round(3.7);         // Round: 4
double power = Math.pow(2, 3);            // 2^3 = 8.0
double squareRoot = Math.sqrt(16);        // Square root: 4.0
double sine = Math.sin(Math.PI / 2);      // Sine of 90 degrees: 1.0

// Converting between types (casting)
double pi = 3.14159;
int truncated = (int) pi;                 // Cast to int: 3 (truncates!)
int fromFloat = (int) 3.9f;               // Still 3, not 4!
double fromInt = (double) 5;              // 5.0
```

**Robotics examples:**
```java
// Calculate wheel RPM from motor RPM with gear ratio
double motorRpm = 5000;
double gearRatio = 8.14;
double wheelRpm = motorRpm / gearRatio;  // About 614 RPM

// Convert inches to meters using WPILib Units class
import edu.wpi.first.math.util.Units;
double heightInches = 78.5;
double heightMeters = Units.inchesToMeters(heightInches);  // About 1.99 meters
```

**In our code:** See `src_java/frc/robot/Constants.java` for `Units.inchesToMeters()` usage.

---

## 1.3 Printing and Seeing Output

Print statements help you debug by showing values while code runs.

```java
// Basic print
System.out.println("Hello!");            // Prints with newline
System.out.print("Hello ");              // Prints without newline
System.out.print("World!");              // Output: Hello World!

// Print variables
int motorId = 9;
System.out.println("Motor ID is: " + motorId);  // String concatenation
System.out.println(42);                  // Output: 42
System.out.println(true);                // Output: true

// String concatenation with +
double height = 45.5;
String name = "elevator";
System.out.println("The " + name + " height is " + height + " inches");
// Output: The elevator height is 45.5 inches

// String.format() - more control over formatting
System.out.println(String.format("Height: %.2f", 3.14159));  // Height: 3.14
System.out.println(String.format("Count: %,d", 1234567));    // Count: 1,234,567
System.out.println(String.format("Padded: %05d", 42));       // Padded: 00042

// printf - shortcut for formatted printing
System.out.printf("Pi is %.4f%n", 3.14159);  // Pi is 3.1416 (%n is newline)
System.out.printf("%s height: %.1f%n", name, height);  // elevator height: 45.5

// Common format specifiers:
// %d - integer
// %f - floating point
// %.2f - floating point with 2 decimal places
// %s - string
// %b - boolean
// %n - newline (platform-independent)
```

**In our code:** We use SmartDashboard instead of print for robot display:
```java
SmartDashboard.putNumber("Elevator/Height", getCarriageHeight());
SmartDashboard.putBoolean("Claw/HasPossession", hasPossession());
```

---

## 1.4 Comments - Notes in Code

Comments are notes for humans. Java ignores them completely.

```java
// This is a single-line comment - starts with //
int motorId = 9;  // Comments can go at the end of a line too

// Use comments to explain WHY, not WHAT
// BAD: Set motorId to 9
// GOOD: Left elevator motor - see wiring diagram

/*
 * This is a multi-line comment.
 * Use it for longer explanations.
 * The asterisks on each line are optional but look nice.
 */

/*
 * You can also use this style to temporarily
 * disable blocks of code during testing.
 */

// TODO comments mark things to fix later
// TODO: Add error handling here
// FIXME: This calculation seems wrong
// NOTE: This assumes positive values only
```

**Javadoc Comments - Documentation:**
```java
/**
 * Calculate speed from distance and time.
 * <p>
 * This method uses the basic physics formula: speed = distance / time.
 *
 * @param distance The distance traveled in meters
 * @param time The time taken in seconds
 * @return Speed in meters per second
 * @throws IllegalArgumentException if time is zero
 */
public double calculateSpeed(double distance, double time) {
    if (time == 0) {
        throw new IllegalArgumentException("Time cannot be zero");
    }
    return distance / time;
}
```

**In our code:** See Javadoc comments throughout `src_java/frc/robot/subsystems/` explaining method purposes.

---

# Part 2: Making Decisions

## 2.1 If Statements

If statements let your code make decisions based on conditions.

**CRITICAL: Use curly braces `{}` for code blocks!** Unlike Python's indentation, Java uses braces to define blocks.

```java
double height = 50;

// Basic if - runs the code in braces only if condition is true
if (height > 40) {
    System.out.println("High position");
    System.out.println("Be careful!");
}
System.out.println("This always runs");  // Outside the if block

// If-else - do one thing or another
if (height > 40) {
    System.out.println("High position");
} else {
    System.out.println("Low position");
}

// If-else if-else - check multiple conditions
if (height > 70) {
    System.out.println("Level 4");
} else if (height > 40) {
    System.out.println("Level 3");
} else if (height > 30) {
    System.out.println("Level 2");
} else {
    System.out.println("Level 1");
}

// Single-line if (braces optional but recommended)
if (height > 50) System.out.println("High");  // Works but not recommended

// Nested if statements
boolean hasPiece = true;
boolean atTarget = true;

if (hasPiece) {
    System.out.println("We have a game piece");
    if (atTarget) {
        System.out.println("Ready to score!");
    } else {
        System.out.println("Moving to target...");
    }
} else {
    System.out.println("Need to pick up a piece");
}
```

**In our code:** See `src_java/frc/robot/Robot.java` for checking if autonomous command exists.

---

## 2.2 Comparisons and Boolean Logic

Comparisons create true/false values that control if statements.

```java
// Comparison operators - all return true or false
int x = 10;
int y = 5;

boolean equal = (x == y);       // Equal to: false
boolean notEqual = (x != y);    // Not equal to: true
boolean greater = (x > y);      // Greater than: true
boolean less = (x < y);         // Less than: false
boolean greaterEq = (x >= y);   // Greater than or equal: true
boolean lessEq = (x <= y);      // Less than or equal: false

// COMMON MISTAKE: = vs ==
x = 5;        // Assignment: puts 5 into x
x == 5;       // Comparison: checks if x equals 5, returns true/false

// Comparing strings - use .equals(), NOT ==
String name = "elevator";
boolean same = name.equals("elevator");     // true
boolean wrong = (name == "elevator");       // WRONG! May not work as expected
boolean ignoreCase = name.equalsIgnoreCase("ELEVATOR");  // true

// Boolean operators - combine multiple conditions
double height = 50;
double speed = 0.5;

// AND (&&) - both must be true
if (height > 40 && speed < 1.0) {
    System.out.println("High and slow");
}

// OR (||) - at least one must be true
if (height > 70 || speed > 1.0) {
    System.out.println("Either very high or fast");
}

// NOT (!) - flips true to false and vice versa
boolean isDisabled = false;
if (!isDisabled) {
    System.out.println("System is enabled");
}

// Combining multiple conditions (use parentheses for clarity)
if ((height > 30 && height < 60) || speed == 0) {
    System.out.println("In safe range or stopped");
}

// Short-circuit evaluation
// Java stops evaluating as soon as it knows the result
if (x != 0 && 10 / x > 2) {  // Won't divide if x is 0
    System.out.println("Safe division");
}
```

**In our code:** See `src_java/frc/robot/Superstructure.java` for complex boolean conditions.

---

## 2.3 Ternary Operator

A one-line shortcut for simple if-else statements.

```java
// Long way
double speed = 0.8;
int direction;
if (speed > 0) {
    direction = 1;
} else {
    direction = -1;
}

// Short way (ternary operator)
// Format: condition ? valueIfTrue : valueIfFalse
int dir = speed > 0 ? 1 : -1;

// More examples
String status = isActive ? "enabled" : "disabled";
String message = height > 50 ? "High" : "Low";

// Can be used anywhere you need a value
System.out.println("Direction: " + (speed > 0 ? 1 : -1));

// Can be chained (but gets hard to read!)
String level = height > 70 ? "high" : height > 40 ? "medium" : "low";
// Better to use regular if-else for complex logic
```

**In our code:** See `src_java/frc/robot/OI.java`:
```java
public static double deadband(double value, double band) {
    return Math.abs(value) > band ? value : 0;
}
```

---

# Part 3: Collections - Storing Multiple Things

## 3.1 Arrays - Fixed-Size Collections

Arrays hold multiple items of the same type. Size is fixed at creation.

```java
// Creating arrays
double[] heights = {30.5, 31, 29.75, 45.25, 78.5};
int[] motorIds = {9, 10, 11, 12};
String[] names = {"elevator", "arm", "claw"};
double[] empty = new double[5];  // Array of 5 zeros

// Accessing items by index (starts at 0!)
// Index:        0      1      2       3      4
// heights = [30.5,   31,  29.75,  45.25,  78.5]
double first = heights[0];          // 30.5
double second = heights[1];         // 31
double last = heights[heights.length - 1];  // 78.5

// Modifying arrays
heights[0] = 32.0;                  // Change first item

// Array info
int length = heights.length;        // 5 (note: no parentheses!)

// Check if value exists (manual loop required)
boolean found = false;
for (double h : heights) {
    if (h == 45.25) {
        found = true;
        break;
    }
}

// Iterating over arrays
for (int i = 0; i < heights.length; i++) {
    System.out.println("Index " + i + ": " + heights[i]);
}

// Enhanced for loop (foreach)
for (double height : heights) {
    System.out.println(height);
}

// Multi-dimensional arrays
double[][] grid = {
    {1.0, 2.0, 3.0},
    {4.0, 5.0, 6.0}
};
double value = grid[1][2];  // 6.0 (row 1, column 2)
```

**In our code:** See `src_java/frc/robot/Constants.java` for array usage in feedforward constants.

---

## 3.2 ArrayList - Dynamic Collections

ArrayList is a resizable array. Much more flexible than regular arrays.

```java
import java.util.ArrayList;
import java.util.List;

// Creating ArrayLists
ArrayList<Double> heights = new ArrayList<>();  // Empty list
List<String> names = new ArrayList<>();         // Using interface type (preferred)

// ArrayList with initial values
List<Integer> ids = new ArrayList<>(List.of(9, 10, 11, 12));

// Adding items
heights.add(30.5);          // Add to end
heights.add(0, 25.0);       // Add at index 0

// Accessing items
double first = heights.get(0);
double last = heights.get(heights.size() - 1);

// Modifying items
heights.set(0, 32.0);       // Replace item at index 0

// Removing items
heights.remove(0);          // Remove at index
heights.remove(Double.valueOf(30.5));  // Remove by value

// Size and checking
int size = heights.size();
boolean isEmpty = heights.isEmpty();
boolean contains = heights.contains(45.25);
int index = heights.indexOf(45.25);  // -1 if not found

// Iterating
for (Double height : heights) {
    System.out.println(height);
}

// With index
for (int i = 0; i < heights.size(); i++) {
    System.out.println("Index " + i + ": " + heights.get(i));
}

// Clear all items
heights.clear();
```

**Important:** ArrayList uses wrapper classes (Integer, Double) not primitives (int, double).

**In our code:** See `src_java/frc/robot/subsystems/Vision.java` for ArrayList of pose estimators.

---

## 3.3 HashMap - Key-Value Pairs

HashMap stores data with unique keys for fast lookup.

```java
import java.util.HashMap;
import java.util.Map;

// Creating HashMaps
Map<String, Integer> motorIds = new HashMap<>();
Map<String, Double> constants = new HashMap<>();

// Adding entries
motorIds.put("elevator_left", 9);
motorIds.put("elevator_right", 10);
motorIds.put("claw", 11);

// Getting values
int clawId = motorIds.get("claw");  // 11
int unknown = motorIds.getOrDefault("unknown", -1);  // -1 (default if key missing)

// Checking
boolean hasKey = motorIds.containsKey("claw");
boolean hasValue = motorIds.containsValue(11);
int size = motorIds.size();

// Removing
motorIds.remove("claw");

// Iterating over keys
for (String name : motorIds.keySet()) {
    System.out.println(name + " -> " + motorIds.get(name));
}

// Iterating over entries (more efficient)
for (Map.Entry<String, Integer> entry : motorIds.entrySet()) {
    System.out.println(entry.getKey() + " -> " + entry.getValue());
}

// Creating with initial values
Map<String, Double> speeds = Map.of(
    "slow", 0.3,
    "normal", 0.7,
    "fast", 1.0
);
// Note: Map.of() creates an immutable map!
```

---

# Part 4: Loops - Doing Things Repeatedly

## 4.1 For Loops

For loops run code a specific number of times.

```java
// Basic for loop
// for (initialization; condition; update)
for (int i = 0; i < 5; i++) {
    System.out.println("Count: " + i);  // Prints 0, 1, 2, 3, 4
}

// Count backwards
for (int i = 5; i > 0; i--) {
    System.out.println(i);  // Prints 5, 4, 3, 2, 1
}

// Skip by 2
for (int i = 0; i < 10; i += 2) {
    System.out.println(i);  // Prints 0, 2, 4, 6, 8
}

// Enhanced for loop (foreach) - iterate over collections
double[] heights = {30.5, 31, 29.75, 45.25, 78.5};
for (double height : heights) {
    System.out.println(height);
}

// Loop with index when you need it
String[] positions = {"A", "B", "C", "D"};
for (int i = 0; i < positions.length; i++) {
    System.out.println("Position " + i + ": " + positions[i]);
}
```

**In our code:** See `src_java/frc/robot/RobotContainer.java` for looping over reef positions.

---

## 4.2 While Loops

While loops run as long as a condition is true.

```java
// Basic while loop
int count = 0;
while (count < 5) {
    System.out.println(count);
    count++;  // Don't forget to update or you'll loop forever!
}

// Do-while loop - runs at least once
int value = 10;
do {
    System.out.println("Value: " + value);
    value++;
} while (value < 5);  // Condition checked after first run

// Breaking out of a loop
while (true) {  // Infinite loop
    double reading = getSensorReading();
    if (reading > 100) {
        break;  // Exit the loop immediately
    }
}

// Skipping an iteration
for (int i = 0; i < 10; i++) {
    if (i % 2 == 0) {
        continue;  // Skip to next iteration
    }
    System.out.println(i);  // Only prints odd numbers
}
```

---

# Part 5: Methods - Reusable Code Blocks

## 5.1 Defining Methods

Methods are reusable blocks of code that perform a specific task.

```java
// Method syntax:
// accessModifier returnType methodName(parameters) { body }

// Method that returns nothing (void)
public void printMessage() {
    System.out.println("Hello!");
}

// Method that returns a value
public double calculateSpeed(double distance, double time) {
    return distance / time;
}

// Method with no parameters
public double getMaxHeight() {
    return 78.5;
}

// Multiple parameters
public double clamp(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// Calling methods
printMessage();                           // Hello!
double speed = calculateSpeed(100, 20);   // 5.0
double clamped = clamp(150, 0, 100);      // 100.0

// Method overloading - same name, different parameters
public int add(int a, int b) {
    return a + b;
}

public double add(double a, double b) {
    return a + b;
}

public int add(int a, int b, int c) {
    return a + b + c;
}
```

**Access modifiers:**
- `public` - accessible from anywhere
- `private` - only accessible within the same class
- `protected` - accessible within package and subclasses
- (none) - package-private, accessible within the same package

**In our code:** See `src_java/frc/robot/subsystems/Elevator.java` for method examples.

---

## 5.2 Static Methods

Static methods belong to the class, not instances. Called with ClassName.methodName().

```java
public class MathHelper {
    // Static method - no instance needed
    public static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }

    // Non-static method - needs an instance
    public double convertInches(double inches) {
        return inches * 0.0254;
    }
}

// Calling static method (no object needed)
double meters = MathHelper.inchesToMeters(78.5);

// Calling non-static method (needs object)
MathHelper helper = new MathHelper();
double meters2 = helper.convertInches(78.5);
```

**In our code:** See `src_java/frc/robot/OI.java` for static `deadband()` method.

---

# Part 6: Classes and Objects

## 6.1 Classes - Blueprints for Objects

Classes define what an object contains (fields) and can do (methods).

```java
public class Motor {
    // Fields (instance variables)
    private int id;
    private double speed;
    private boolean isRunning;

    // Constructor - called when creating a new object
    public Motor(int id) {
        this.id = id;      // 'this' refers to the current object
        this.speed = 0;
        this.isRunning = false;
    }

    // Getter methods
    public int getId() {
        return id;
    }

    public double getSpeed() {
        return speed;
    }

    public boolean isRunning() {
        return isRunning;
    }

    // Setter methods
    public void setSpeed(double speed) {
        this.speed = speed;
    }

    // Other methods
    public void start() {
        isRunning = true;
    }

    public void stop() {
        isRunning = false;
        speed = 0;
    }
}

// Creating objects (instances)
Motor leftMotor = new Motor(9);
Motor rightMotor = new Motor(10);

// Using objects
leftMotor.setSpeed(0.5);
leftMotor.start();
System.out.println(leftMotor.getSpeed());  // 0.5
leftMotor.stop();
```

**In our code:** Every subsystem is a class - see `src_java/frc/robot/subsystems/Elevator.java`.

---

## 6.2 Inheritance - Extending Classes

Inheritance lets you create new classes based on existing ones.

```java
// Base class (parent/superclass)
public class GamePiece {
    protected String name;
    protected double weight;

    public GamePiece(String name, double weight) {
        this.name = name;
        this.weight = weight;
    }

    public void pickup() {
        System.out.println("Picking up " + name);
    }
}

// Derived class (child/subclass)
public class Coral extends GamePiece {
    private String color;

    public Coral(String color, double weight) {
        super("Coral", weight);  // Call parent constructor
        this.color = color;
    }

    // Override parent method
    @Override
    public void pickup() {
        System.out.println("Carefully picking up " + color + " coral");
    }

    // New method only in Coral
    public String getColor() {
        return color;
    }
}

// Using inheritance
GamePiece piece = new Coral("orange", 0.5);
piece.pickup();  // "Carefully picking up orange coral"
```

**In our code:** All subsystems extend `SubsystemBase`:
```java
public class Elevator extends SubsystemBase {
    // ...
}
```

---

## 6.3 Interfaces - Contracts for Classes

Interfaces define what methods a class must implement.

```java
// Interface definition
public interface DriverActionSet {
    double forward();       // Abstract method - no body
    double strafe();
    double turn();
    Trigger resetGyro();
    boolean isMovementCommanded();
}

// Class implementing the interface
public class XboxDriver implements DriverActionSet {
    private CommandXboxController stick;

    public XboxDriver(int port) {
        stick = new CommandXboxController(port);
    }

    @Override
    public double forward() {
        return -stick.getLeftY();
    }

    @Override
    public double strafe() {
        return -stick.getLeftX();
    }

    @Override
    public double turn() {
        return -stick.getRightX();
    }

    @Override
    public Trigger resetGyro() {
        return stick.start();
    }

    @Override
    public boolean isMovementCommanded() {
        return forward() + strafe() + turn() != 0;
    }
}

// Using interface as type (polymorphism)
DriverActionSet driver = new XboxDriver(0);
double fwd = driver.forward();
```

**In our code:** See `src_java/frc/robot/OI.java` for interface definitions.

---

# Part 7: FRC/WPILib Specific Concepts

## 7.1 Subsystems

Subsystems represent individual robot mechanisms.

```java
public class Elevator extends SubsystemBase {
    private final SparkMax leader;
    private final RelativeEncoder encoder;

    public Elevator() {
        leader = new SparkMax(9, MotorType.kBrushless);
        encoder = leader.getEncoder();
    }

    // Called every 20ms
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator/Height", getHeight());
    }

    public void setHeight(double height) {
        // Set motor to reach height
    }

    public double getHeight() {
        return encoder.getPosition();
    }
}
```

---

## 7.2 Commands

Commands are actions that use subsystems.

```java
// Simple inline command
Command raiseElevator = Commands.run(() -> elevator.setHeight(1.0), elevator);

// Command class
public class SetHeightCommand extends Command {
    private final Elevator elevator;
    private final double targetHeight;

    public SetHeightCommand(double height, Elevator elevator) {
        this.elevator = elevator;
        this.targetHeight = height;
        addRequirements(elevator);  // This command needs the elevator
    }

    @Override
    public void initialize() {
        // Runs once when command starts
    }

    @Override
    public void execute() {
        // Runs every 20ms while command is active
        elevator.setHeight(targetHeight);
    }

    @Override
    public boolean isFinished() {
        // Return true when command should end
        return elevator.atHeight(targetHeight);
    }

    @Override
    public void end(boolean interrupted) {
        // Runs once when command ends
    }
}

// Using commands
Command cmd = new SetHeightCommand(1.5, elevator);
cmd.schedule();  // Start the command
```

---

## 7.3 Command Composition

Combine commands into complex sequences.

```java
import edu.wpi.first.wpilibj2.command.Commands;

// Sequential - run one after another
Command sequence = Commands.sequence(
    Commands.runOnce(() -> System.out.println("First")),
    Commands.waitSeconds(1.0),
    Commands.runOnce(() -> System.out.println("Second"))
);

// Parallel - run all at once, end when all finish
Command parallel = Commands.parallel(
    elevator.setHeightCommand(1.5),
    arm.setAngleCommand(Rotation2d.fromDegrees(-35))
);

// ParallelRace - run all at once, end when ANY finishes
Command race = Commands.race(
    claw.intakeCommand(),
    Commands.waitSeconds(3.0)  // Timeout after 3 seconds
);

// Chaining commands
Command auto = elevator.setHeightCommand(1.5)
    .andThen(arm.setAngleCommand(Rotation2d.fromDegrees(-35)))
    .andThen(claw.outtakeCommand().withTimeout(1.5));

// Conditional command
Command conditional = Commands.either(
    elevator.setHeightCommand(1.5),  // If true
    elevator.setHeightCommand(0.5),  // If false
    () -> isHighScore                 // Condition
);
```

---

## 7.4 Triggers and Button Bindings

Connect controller buttons to commands.

```java
// In RobotContainer constructor
CommandXboxController controller = new CommandXboxController(0);

// Button triggers
controller.a().onTrue(elevator.setHeightCommand(1.0));      // When pressed
controller.b().whileTrue(claw.intakeCommand());              // While held
controller.x().toggleOnTrue(lightCommand);                    // Toggle
controller.y().onFalse(Commands.runOnce(() -> stop()));     // When released

// Custom trigger from any boolean supplier
new Trigger(() -> elevator.getHeight() > 1.0)
    .onTrue(Commands.print("Elevator high!"));

// Combining triggers
controller.a().and(controller.b())
    .onTrue(specialCommand);  // Both A AND B pressed

controller.leftBumper().or(controller.rightBumper())
    .whileTrue(boostCommand);  // Either bumper held
```

---

# Part 8: Lambda Expressions

## 8.1 What Are Lambdas?

Lambdas are compact anonymous functions, great for short inline code.

```java
// Traditional way with anonymous class
Runnable oldWay = new Runnable() {
    @Override
    public void run() {
        System.out.println("Hello!");
    }
};

// Lambda way
Runnable newWay = () -> System.out.println("Hello!");

// Lambda syntax:
// (parameters) -> expression
// (parameters) -> { statements; }

// Examples with different parameter counts
Runnable noParams = () -> System.out.println("No params");
Consumer<String> oneParam = s -> System.out.println(s);
BiFunction<Integer, Integer, Integer> twoParams = (a, b) -> a + b;

// Multi-line lambdas use braces
Runnable multiLine = () -> {
    System.out.println("Line 1");
    System.out.println("Line 2");
};
```

**In our code:** Lambdas are used extensively for commands:
```java
Command teleopDrive = swerve.teleopCommand(
    () -> applySpeedCurve(driverJoystick.forward()),  // Lambda for forward
    () -> applySpeedCurve(driverJoystick.strafe()),   // Lambda for strafe
    () -> applySpeedCurve(driverJoystick.turn())      // Lambda for turn
);
```

---

## 8.2 Method References

Method references are shortcuts for lambdas that just call an existing method.

```java
// Lambda
Consumer<String> lambda = s -> System.out.println(s);

// Method reference (equivalent)
Consumer<String> methodRef = System.out::println;

// Types of method references:
// 1. Static method: ClassName::methodName
Function<Double, Double> abs = Math::abs;

// 2. Instance method on object: object::methodName
String str = "hello";
Supplier<Integer> len = str::length;

// 3. Instance method on parameter: ClassName::methodName
Function<String, String> upper = String::toUpperCase;

// 4. Constructor: ClassName::new
Supplier<ArrayList<String>> constructor = ArrayList::new;

// In FRC code
controller.start().onTrue(Commands.runOnce(swerve::resetGyro));
controller.a().whileTrue(Commands.run(claw::intake, claw));
```

---

# Part 9: Best Practices

## 9.1 Code Organization

```java
// Constants in a separate class with nested classes
public final class Constants {
    public static final class ElevatorConstants {
        public static final int MOTOR_ID = 9;
        public static final double MAX_HEIGHT = 1.5;
    }

    public static final class ClawConstants {
        public static final int MOTOR_ID = 11;
    }
}

// Using constants
int id = Constants.ElevatorConstants.MOTOR_ID;
```

---

## 9.2 Error Handling

```java
// Try-catch blocks
try {
    double result = 10 / 0;  // This will throw an exception
} catch (ArithmeticException e) {
    System.out.println("Cannot divide by zero!");
}

// Multiple catch blocks
try {
    // risky code
} catch (IOException e) {
    System.out.println("IO error: " + e.getMessage());
} catch (Exception e) {
    System.out.println("Other error: " + e.getMessage());
}

// Finally block (always runs)
try {
    // risky code
} catch (Exception e) {
    // handle error
} finally {
    // cleanup code - always runs
}

// Throwing exceptions
public void setHeight(double height) {
    if (height < 0 || height > MAX_HEIGHT) {
        throw new IllegalArgumentException("Height out of range: " + height);
    }
    // set height
}
```

---

# Part 10: Practice Exercises

## Exercise 1: Variables and Math
Create a method that converts motor RPM to wheel RPM given a gear ratio.
```java
// Fill in the method
public double motorToWheelRpm(double motorRpm, double gearRatio) {
    // Your code here
}
// motorToWheelRpm(5000, 8.14) should return about 614
```

## Exercise 2: Conditionals
Create a method that returns the scoring level (1-4) based on elevator height.
```java
// Levels: 1 (<35"), 2 (35-45"), 3 (45-70"), 4 (>70")
public int getLevel(double heightInches) {
    // Your code here
}
```

## Exercise 3: Arrays
Create a method that finds the average of an array of sensor readings.
```java
public double averageReading(double[] readings) {
    // Your code here
}
```

## Exercise 4: ArrayList
Create a method that keeps only the last N readings in an ArrayList.
```java
public void trimToSize(ArrayList<Double> readings, int maxSize) {
    // Your code here
}
```

## Exercise 5: Classes
Create a simple `SensorReading` class with timestamp and value fields.
```java
public class SensorReading {
    // Add fields, constructor, and getters
}
```

## Exercise 6: Inheritance
Create a `LimitedMotor` class that extends a base `Motor` class and prevents speed above 0.8.

## Exercise 7: Interface
Create a `Controllable` interface with `enable()`, `disable()`, and `isEnabled()` methods.

## Exercise 8: Lambda
Rewrite this using a lambda:
```java
button.onTrue(new InstantCommand() {
    @Override
    public void initialize() {
        elevator.reset();
    }
});
```

## Exercise 9: Method Reference
Rewrite using method reference:
```java
list.forEach(item -> System.out.println(item));
```

## Exercise 10: Command Composition
Create a command sequence that:
1. Raises elevator to level 4
2. Waits 0.5 seconds
3. Runs outtake for 1 second
4. Lowers elevator to loading position

## Exercise 11: Static Methods
Create a `MathUtils` class with static methods for common robot math operations.

## Exercise 12: HashMap
Create a method that maps motor names to their CAN IDs using a HashMap.

## Exercise 13: For Loop
Write code that prints all even numbers from 0 to 20.

## Exercise 14: While Loop
Write a sensor polling loop that breaks when a target is detected.

## Exercise 15: Nested Conditions
Write code that determines robot state based on piece possession, target proximity, and elevator position.

## Exercise 16: String Formatting
Format motor telemetry data into a readable string with proper decimal places.

## Exercise 17: Error Handling
Add proper exception handling to a method that calculates gear ratios.

## Exercise 18: Subsystem
Create a basic subsystem class for an LED strip with methods to set color and pattern.

## Exercise 19: Trigger Logic
Create a trigger that activates only when the robot is at the target AND has a game piece.

## Exercise 20: Full Integration
Create a complete scoring sequence command that coordinates elevator, arm, and claw movements with proper timing.

---

# Answer Key

## Exercise 1: Variables and Math
```
The gear ratio represents how many motor rotations
equal one wheel rotation.

Motor RPM: 5000 (input)
Gear Ratio: 8.14 (motor turns per wheel turn)

          Motor RPM      5000
Wheel RPM = ──────────  = ────── ≈ 614.25 RPM
          Gear Ratio     8.14

+──────────────────────────────────────────────+
│                                              │
│   MOTOR ──(8.14:1 gears)──> WHEEL           │
│   5000 RPM                   614 RPM        │
│                                              │
│   For every 8.14 motor rotations,           │
│   the wheel rotates once.                   │
│                                              │
+──────────────────────────────────────────────+
```

```java
public double motorToWheelRpm(double motorRpm, double gearRatio) {
    return motorRpm / gearRatio;
}
```

## Exercise 2: Conditionals
```
Height (inches) ──────────────────────────────────────────>
     0        35         45         70         78.5
     |         |          |          |          |
     +---------+----------+----------+----------+
     | Level 1 | Level 2  | Level 3  | Level 4  |
     +---------+----------+----------+----------+
```

```java
public int getLevel(double heightInches) {
    if (heightInches > 70) {
        return 4;
    } else if (heightInches > 45) {
        return 3;
    } else if (heightInches > 35) {
        return 2;
    } else {
        return 1;
    }
}
```

## Exercise 3: Arrays
```java
public double averageReading(double[] readings) {
    if (readings.length == 0) {
        return 0;
    }
    double sum = 0;
    for (double reading : readings) {
        sum += reading;
    }
    return sum / readings.length;
}
```

## Exercise 4: ArrayList
```java
public void trimToSize(ArrayList<Double> readings, int maxSize) {
    while (readings.size() > maxSize) {
        readings.remove(0);  // Remove oldest (first) reading
    }
}
```

## Exercise 5: Classes
```java
public class SensorReading {
    private final double timestamp;
    private final double value;

    public SensorReading(double timestamp, double value) {
        this.timestamp = timestamp;
        this.value = value;
    }

    public double getTimestamp() {
        return timestamp;
    }

    public double getValue() {
        return value;
    }
}
```

## Exercise 6: Inheritance
```java
public class LimitedMotor extends Motor {
    private static final double MAX_SPEED = 0.8;

    public LimitedMotor(int id) {
        super(id);
    }

    @Override
    public void setSpeed(double speed) {
        double clampedSpeed = Math.min(Math.abs(speed), MAX_SPEED);
        super.setSpeed(speed >= 0 ? clampedSpeed : -clampedSpeed);
    }
}
```

## Exercise 7: Interface
```java
public interface Controllable {
    void enable();
    void disable();
    boolean isEnabled();
}
```

## Exercise 8: Lambda
```java
button.onTrue(Commands.runOnce(() -> elevator.reset()));
```

## Exercise 9: Method Reference
```java
list.forEach(System.out::println);
```

## Exercise 10: Command Composition
```java
Command scoringSequence = Commands.sequence(
    elevator.setHeightCommand(ScoringConstants.L4_HEIGHT),
    Commands.waitSeconds(0.5),
    claw.outtakeCommand().withTimeout(1.0),
    elevator.setHeightCommand(ScoringConstants.LOADING_HEIGHT)
);
```

## Exercise 11: Static Methods
```java
public final class MathUtils {
    private MathUtils() {}  // Prevent instantiation

    public static double deadband(double value, double band) {
        return Math.abs(value) > band ? value : 0;
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }
}
```

## Exercise 12: HashMap
```java
public Map<String, Integer> createMotorMap() {
    Map<String, Integer> motors = new HashMap<>();
    motors.put("elevator_left", 9);
    motors.put("elevator_right", 10);
    motors.put("claw", 11);
    motors.put("arm", 12);
    return motors;
}
```

## Exercise 13: For Loop
```java
for (int i = 0; i <= 20; i += 2) {
    System.out.println(i);
}
```

## Exercise 14: While Loop
```java
while (true) {
    double distance = sensor.getDistance();
    if (distance < TARGET_DISTANCE) {
        System.out.println("Target detected!");
        break;
    }
    Thread.sleep(20);  // Wait 20ms before next poll
}
```

## Exercise 15: Nested Conditions
```java
public String getRobotState(boolean hasPiece, boolean nearTarget, boolean elevatorReady) {
    if (!hasPiece) {
        return "SEEKING_PIECE";
    }
    if (!nearTarget) {
        return "TRAVELING";
    }
    if (!elevatorReady) {
        return "PREPARING";
    }
    return "READY_TO_SCORE";
}
```

## Exercise 16: String Formatting
```java
public String formatTelemetry(int motorId, double voltage, double current, double temp) {
    return String.format("Motor %d: %.2fV, %.2fA, %.1f°C",
        motorId, voltage, current, temp);
}
```

## Exercise 17: Error Handling
```java
public double calculateGearRatio(double inputTeeth, double outputTeeth) {
    if (inputTeeth <= 0 || outputTeeth <= 0) {
        throw new IllegalArgumentException("Tooth counts must be positive");
    }
    return outputTeeth / inputTeeth;
}
```

## Exercise 18: Subsystem
```java
public class LEDStrip extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    public LEDStrip(int port, int length) {
        led = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(length);
        led.setLength(length);
        led.start();
    }

    public void setColor(int r, int g, int b) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, r, g, b);
        }
        led.setData(buffer);
    }

    public void setRainbow() {
        for (int i = 0; i < buffer.getLength(); i++) {
            int hue = (i * 180 / buffer.getLength()) % 180;
            buffer.setHSV(i, hue, 255, 128);
        }
        led.setData(buffer);
    }
}
```

## Exercise 19: Trigger Logic
```java
Trigger readyToScore = new Trigger(() ->
    autoAlign.isAtGoalPose() && claw.hasPossession()
);

readyToScore.onTrue(Commands.print("Ready to score!"));
```

## Exercise 20: Full Integration
```java
public Command fullScoringSequence(int level) {
    return Commands.sequence(
        // Move to approach position
        Commands.parallel(
            level == 4 ? elevator.setHeightCommand(ScoringConstants.L4_HEIGHT)
                       : elevator.setHeightCommand(ScoringConstants.L2_HEIGHT),
            coralArm.setAngleCommand(Rotation2d.fromDegrees(-35))
        ),
        // Wait for mechanisms to settle
        Commands.waitUntil(() ->
            elevator.atGoalHeight() && coralArm.atGoalRotation()
        ),
        // Score the piece
        claw.outtakeCommand().withTimeout(1.0),
        // Return to loading position
        Commands.parallel(
            elevator.setHeightCommand(ScoringConstants.LOADING_HEIGHT),
            coralArm.setAngleCommand(Rotation2d.fromDegrees(0))
        )
    ).withName("FullScoringSequence");
}
```

---

## Summary: Python to Java Cheat Sheet

| Concept | Python | Java |
|---------|--------|------|
| Variable | `x = 5` | `int x = 5;` |
| String | `"hello"` or `'hello'` | `"hello"` only |
| Print | `print(x)` | `System.out.println(x);` |
| If | `if x > 5:` | `if (x > 5) { }` |
| For loop | `for i in range(5):` | `for (int i = 0; i < 5; i++) { }` |
| For each | `for item in list:` | `for (Item item : list) { }` |
| While | `while True:` | `while (true) { }` |
| Function/Method | `def func(x):` | `public void func(int x) { }` |
| List | `[1, 2, 3]` | `new int[] {1, 2, 3}` or `ArrayList` |
| Dictionary | `{"a": 1}` | `HashMap<String, Integer>` |
| None/null | `None` | `null` |
| Boolean | `True` / `False` | `true` / `false` |
| String format | `f"Value: {x}"` | `String.format("Value: %d", x)` |
| Lambda | `lambda x: x + 1` | `x -> x + 1` |
| Class | `class Foo:` | `public class Foo { }` |
| Inheritance | `class B(A):` | `class B extends A { }` |
| Interface | Protocol (typing) | `interface` keyword |
| Property | `@property` | getter method |
| Static | N/A (module level) | `static` keyword |
| Comment | `# comment` | `// comment` |
| Multiline comment | `""" comment """` | `/* comment */` |

---

*This guide is specific to Team 3164's 2025 robot code. Good luck learning Java!*
