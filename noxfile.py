
# noxfile.py
import nox


@nox.session
def tests(session):
    """Run unit tests."""
    session.install("pytest", "pytest-cov", ".")
    session.run("pytest")

@nox.session
def lint(session):
    """Run linting with ruff."""
    session.install("ruff")
    session.run("ruff", "check", "src")

@nox.session
def format(session):
    """Format code with black."""
    session.install("black")
    session.run("black", "src", "tests")

@nox.session
def type_check(session):
    """Run type checking with mypy."""
    session.install("mypy", ".")
    session.run("mypy", "src")

@nox.session
def dev(session):
    """Install development dependencies."""
    session.install("-e", ".[dev]")
