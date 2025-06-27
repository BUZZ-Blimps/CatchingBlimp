# Catching Blimp Repository


This is the repo that houses all of the components that go into the CatchingBlimp platform. Currently we have the following structure to this repo:

**BlimpCore**: The software and firmware that will run on the OrangePi aboard the blimp

**VisionModule**: The vision library included as a part of the OnboardCode repo to modularly deploy computer vision models

**TestBench**: The software test suite that will test and validate the blimp system for competition readiness   

**configuration**: useful scripts to setup orangePis, dev laptops, vision, basestation, and more

**documentation**: all documentation and processes for the CatchingBlimp architecture

**deprecated**: old code that is kept got the sake of quick and easy access

# Project Planning Guide

This guide outlines the process for creating and managing issues in our GitHub repository. By following these best practices, we can ensure effective collaboration and efficient project management across software, hardware, and project management teams.

# Turning on a blimp (Updated 6.27.2025)
1. Plug in the battery.
2. Open a terminal
3. `ssh root@opi#` Replace # with the opi number.
4. `./run_catching_blimp.sh` To run controls. 

## Creating an Issue

When creating a new issue, please follow these steps:

1. **Descriptive Title**: Provide a clear and concise title that summarizes the issue.
2. **Issue Description**: In the issue description, include the following information:
   - **Problem Statement**: Clearly describe the problem or task at hand.
   - **Context**: Provide any relevant background information or context.
   - **Steps to Reproduce** (for bugs): If reporting a bug, outline the steps to reproduce the issue.
   - **Expected Behavior** (for bugs): Describe what you expected to happen.
   - **Actual Behavior** (for bugs): Describe what actually happened.
   - **Proposed Solution** (optional): If you have a suggested solution, outline it here.
3. **Labels**: Apply the appropriate labels to categorize the issue:
   - `basestation`: Base station related issue
   - `bug`: Something isn't working as expected
   - `cleanup`: Cleaning or organization required
   - `documentation`: Improvements or additions to documentation
   - `duplicate`: The issue or pull request already exists
   - `enhancement`: New feature or request
   - `good first issue`: Good for newcomers to the project
   - `hardware`: Hardware team issue
   - `help wanted`: Extra attention is needed
   - `invalid`: The issue doesn't seem right or is not applicable
   - `orangepi`: OrangePi related issue
   - `question`: Further information is requested
   - `software`: Software related issue
   - `wontfix`: The issue will not be worked on
4. **Assignee** (optional): If applicable, assign the issue to the relevant team member(s).
5. **Milestone**: Set the appropriate milestone for the issue:
   - **Thursday Datestamp**: Select the milestone with the nearest Thursday datestamp that falls within the next three weeks.
   - `longterm`: For issues without a specific due date or those that are part of a long-term plan.


