import pytest

from ament_flake8.main import main_with_errors


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """Check code style using the package configuration."""
    rc, errors = main_with_errors(
        argv=[
            '--config',
            'setup.cfg',
        ]
    )

    assert rc == 0, (
        'Found %d code style errors / warnings:\n'
        % len(errors)
        + '\n'.join(errors)
    )
