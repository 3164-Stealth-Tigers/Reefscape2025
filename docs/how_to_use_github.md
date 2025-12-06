# How to Use GitHub - A Complete Beginner's Guide

This guide is for students who have never used Git or GitHub before. We'll walk through everything step-by-step on a Windows PC.

## What is Git and GitHub?

```
+-------------------+     +-------------------+
|       Git         |     |      GitHub       |
+-------------------+     +-------------------+
| A tool on your    |     | A website that    |
| computer that     |     | stores your code  |
| tracks changes    |     | online so the     |
| to your code      |     | team can share it |
+-------------------+     +-------------------+
        |                         |
        +--------> Work Together <--------+
```

- **Git** = Software on your computer that tracks code changes
- **GitHub** = Website where the team's code lives online
- **Repository (repo)** = A folder containing all the project's code and history

## Step 1: Install Git on Your PC

### Download Git

1. Go to: https://git-scm.com/download/win
2. Click the download link - it should auto-detect Windows
3. Run the installer

### During Installation

Click "Next" on most screens, but pay attention to these settings:

| Screen | Recommended Setting |
|--------|---------------------|
| Default editor | Select "Use Visual Studio Code" (if you have it) |
| Initial branch name | Choose "Override" and type `main` |
| PATH environment | Choose "Git from the command line and also from 3rd-party software" |
| Everything else | Use the defaults (just click Next) |

### Verify Installation

1. Open **Command Prompt** (search "cmd" in Windows)
2. Type this and press Enter:
   ```
   git --version
   ```
3. You should see something like: `git version 2.43.0`

If you see a version number, Git is installed!

## Step 2: Set Up Your Identity

Git needs to know who you are. Open Command Prompt and run these commands (replace with YOUR information):

```
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

Example:
```
git config --global user.name "John Smith"
git config --global user.email "john.smith@school.edu"
```

## Step 3: Create a GitHub Account

1. Go to https://github.com
2. Click "Sign Up"
3. Follow the steps to create a free account
4. **Important**: Use the same email you used in Step 2!

## Step 4: Download the Team's Code (Clone)

```
+-------------------+                     +-------------------+
|     GitHub        |                     |    Your PC        |
|   (the cloud)     |                     |   (your copy)     |
|                   |                     |                   |
|  Team's Code      | ---- Clone ---->    |  Your Copy of     |
|  (original)       |                     |  the Code         |
+-------------------+                     +-------------------+
```

### Steps to Clone

1. Open Command Prompt
2. Navigate to where you want the code. For example, to put it in Documents:
   ```
   cd Documents
   ```
3. Clone the repository:
   ```
   git clone https://github.com/Team3164/Reefscape2025.git
   ```
4. Enter the project folder:
   ```
   cd Reefscape2025
   ```

You now have all the team's code on your computer!

## Step 5: Create Your Own Branch

**What is a branch?**

Think of branches like making a photocopy of a document. You can write on your copy without changing the original.

```
main branch (the original)
    |
    +---> your-branch (your copy to work on)
           |
           | Make changes here safely!
           |
    +------+ (later: merge back when ready)
    |
main branch (original + your changes)
```

### Create a New Branch

1. Make sure you're in the project folder
2. First, get the latest code:
   ```
   git pull
   ```
3. Create and switch to your new branch:
   ```
   git checkout -b your-name/what-youre-doing
   ```

Example:
```
git checkout -b john/fix-elevator-speed
```

**Branch naming tips:**
- Use your name and what you're working on
- Use dashes instead of spaces
- Keep it short but descriptive

## Step 6: Make Your Changes

Now you can edit the code! Use your code editor (VS Code recommended).

### Open the Project in VS Code

```
code .
```

Or open VS Code and use File > Open Folder to select the Reefscape2025 folder.

### Make a Simple Change

For your first time, try something simple like adding a comment or fixing a typo. For example, edit `src/constants.py` and add a comment.

## Step 7: Check What You Changed

Before committing, see what files you modified:

```
git status
```

This shows:
- **Red files** = Changed but not staged
- **Green files** = Staged and ready to commit

To see the actual changes in detail:
```
git diff
```

## Step 8: Stage and Commit Your Changes

```
+-------------+     +-------------+     +-------------+
|   Working   |     |   Staging   |     |   Commit    |
|  Directory  | --> |    Area     | --> |   History   |
+-------------+     +-------------+     +-------------+
  Your edits         Ready to save       Saved snapshot
```

### Stage Your Changes (Add to Staging Area)

To stage specific files:
```
git add filename.py
```

To stage ALL changed files:
```
git add .
```

### Commit (Save the Snapshot)

```
git commit -m "A short description of what you changed"
```

Example:
```
git commit -m "Fix elevator max height to prevent collision"
```

**Good commit messages:**
- Start with a verb: "Fix", "Add", "Update", "Remove"
- Keep it under 50 characters
- Describe WHAT you did, not HOW

## Step 9: Push Your Branch to GitHub

Your changes are saved locally. Now share them with the team:

```
git push -u origin your-branch-name
```

Example:
```
git push -u origin john/fix-elevator-speed
```

The first time, Git might ask you to log into GitHub. Follow the prompts.

## Step 10: Create a Pull Request (PR)

A Pull Request asks the team to review and merge your changes into the main code.

```
+-------------------+        +-------------------+
|    Your Branch    | -----> |   Pull Request    |
+-------------------+        +-------------------+
                                     |
                             Team Reviews & Approves
                                     |
                                     v
                            +-------------------+
                            |    Main Branch    |
                            | (everyone's code) |
                            +-------------------+
```

### Create the PR on GitHub

1. Go to https://github.com/Team3164/Reefscape2025
2. You'll see a yellow banner saying "your-branch had recent pushes"
3. Click the green **"Compare & pull request"** button

### Fill Out the PR Form

```
+--------------------------------------------------+
| Title: [Short description of your change]        |
+--------------------------------------------------+
| Description:                                      |
|                                                  |
| - What did you change?                           |
| - Why did you change it?                         |
| - How can someone test it?                       |
|                                                  |
+--------------------------------------------------+
```

4. Write a clear title and description
5. Click **"Create pull request"**

That's it! A mentor or team lead will review your code and either:
- **Approve and merge it** - Your code is now part of the project!
- **Request changes** - They'll leave comments on what to fix

## Quick Reference Cheat Sheet

| What You Want to Do | Command |
|---------------------|---------|
| Check Git version | `git --version` |
| Clone the repo | `git clone https://github.com/Team3164/Reefscape2025.git` |
| Get latest changes | `git pull` |
| Create new branch | `git checkout -b branch-name` |
| Switch to a branch | `git checkout branch-name` |
| See what branch you're on | `git branch` |
| See what you changed | `git status` |
| See detailed changes | `git diff` |
| Stage files | `git add filename` or `git add .` |
| Commit changes | `git commit -m "message"` |
| Push to GitHub | `git push` |
| Push new branch | `git push -u origin branch-name` |

## Common Problems and Solutions

### "I made changes on the wrong branch!"

Don't panic! You can move your changes:
```
git stash
git checkout correct-branch-name
git stash pop
```

### "I need to undo my last commit"

```
git reset --soft HEAD~1
```
This keeps your changes but undoes the commit.

### "My branch is behind main"

Get the latest main and merge it into your branch:
```
git checkout main
git pull
git checkout your-branch-name
git merge main
```

### "Git says I have merge conflicts"

This means you and someone else changed the same lines. Open the file and look for:
```
<<<<<<< HEAD
Your changes
=======
Their changes
>>>>>>> main
```

Edit the file to keep what you want, remove the `<<<<`, `====`, and `>>>>` markers, then:
```
git add .
git commit -m "Resolve merge conflicts"
```

### "I forgot to make a branch and committed to main!"

```
git branch new-branch-name
git reset --hard HEAD~1
git checkout new-branch-name
```

## The Complete Workflow (Summary)

```
1. git pull                    (get latest code)
         |
         v
2. git checkout -b my-branch   (create your branch)
         |
         v
3. [Make your changes in VS Code]
         |
         v
4. git add .                   (stage changes)
         |
         v
5. git commit -m "message"     (save changes)
         |
         v
6. git push -u origin my-branch (upload to GitHub)
         |
         v
7. [Create Pull Request on GitHub website]
         |
         v
8. [Wait for review and merge]
```

## Getting Help

- **GitHub Docs**: https://docs.github.com/en/get-started
- **Ask a teammate or mentor** - We've all been beginners!
- **Google the error message** - Someone else has probably had the same problem

---

*Remember: Everyone makes mistakes with Git at first. The best way to learn is to practice. Don't be afraid to ask for help!*
