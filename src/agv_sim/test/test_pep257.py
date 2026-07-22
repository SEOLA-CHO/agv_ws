import pytest

from ament_pep257.main import main


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    """Check Python documentation style."""
    rc = main(
        argv=[
            '.',
            'test',
            '--add-ignore',
            'D202',
        ]
    )

    assert rc == 0, 'Found code style errors / warnings'
