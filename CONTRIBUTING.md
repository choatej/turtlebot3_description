# Contribution Guide

Thank you for considering contributing to the Turtlebot3 Description project! Contributions are what make open-source projects thrive, and your help is greatly appreciated.

Below is a guide on how you can contribute to this ROS2 project. Whether you're fixing a bug, adding a new feature, or improving documentation, this guide will help you get started.

## Table of Contents
* [How to Contribute](#how-to-contribute)
* [Pull Request Process](#pull-request-process)
* [Code Style](#code-style)
* [Running Tests](#)
* [Reporting Bugs](#reporting-bugs)
* [Feature Requests](#feature-requests)
* [License](#license)


## How to Contribute

### 1. Fork the Repository
Fork this repository to your own GitHub account by clicking the "Fork" button at the top-right of this repository's page.

### 2. Clone Your Fork Locally
Once you have your fork, clone it locally to your machine:

```bash
git clone https://github.com/<your-username>/turtlebot3_description.git
cd turtlebot3_description
```
### 3. Create a Branch
Create a new branch to work on your changes. Make sure your branch name is descriptive of the change you're making:

```bash
git checkout -b feature/new-feature-name
```
### 4. Make Your Changes
Edit the files and add your changes. Ensure that you follow the code style and standards laid out in this guide.

### 5. Commit Your Changes
Once you're happy with your changes, commit them to your branch:

```bash
git add .
git commit -m "Add brief description of your changes"
```

### 6. Push Your Changes
Push your changes to your forked repository:

```bash
git push origin feature/new-feature-name
```

### 7. Submit a Pull Request
Open a pull request (PR) from your branch to the main branch of the original repository. In your PR description, clearly explain the purpose of your changes and reference any related issues, if applicable.

## Pull Request Process
1. Ensure your code passes all CI checks: We use GitHub Actions for Continuous Integration (CI). Your PR should pass all CI checks (e.g., linting, formatting, and tests).
1. Follow the PR template: Provide a detailed description of the change and link to any issues being addressed.
1. Keep PRs focused: Try to keep your PRs focused on one feature or bug fix. This makes it easier to review and integrate.
1. Await review: Your PR will be reviewed by project maintainers. Please be patient, and feel free to respond to any feedback.
1. Merge: Once approved, the PR will be merged by a maintainer. After merging, your branch will be deleted.

## Code Style
This project follows standard ROS2 coding conventions and uses ament linters for consistency. Ensure you are familiar with the ROS2 style guides before submitting your contributions.

**Python:**  Follow PEP8 guidelines for Python code.

**C++:** Follow the ROS2 C++ style guide.

**XML:** Ensure that your URDF and launch files are well-formatted and valid XML.

## Linting
Before committing your code, run the ROS2 linters to ensure that your code adheres to the style guides:

```bash
colcon test --packages-select turtlebot3_description
colcon test-result --all --verbose
```
If you see any warnings or errors, please address them before submitting your PR.

## Running Tests

### Unit Tests
Run the unit tests locally to ensure your changes don’t break any functionality:

```bash
colcon build --packages-select turtlebot3_description
colcon test --packages-select turtlebot3_description
```

### Integration Tests
If applicable, include any new tests for features you add. Make sure to document how these tests work.

## Reporting Bugs
If you find a bug, please open an issue in the GitHub repository. Be sure to include:

- A clear, descriptive title.
- Steps to reproduce the issue.
- The expected outcome.
- What actually happens.
- Any relevant logs or error messages.

## Feature Requests

If you have an idea for a feature you'd like to see in this project, please open an issue to discuss it. We'd love to hear from you!

When creating a feature request, please include:

- A description of the feature.
- Why this feature would be useful.
- How you imagine it being implemented.

## License

By contributing, you agree that your contributions will be licensed under the MIT License.

---

That's it! Thanks again for considering contributing to this project. If you have any questions, feel free to reach out by opening an issue.

