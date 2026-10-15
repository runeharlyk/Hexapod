"""The secrets.h check that runs before every firmware build (pre_build.py).

    uv run --no-project --with pytest pytest firmware/scripts
"""
from pathlib import Path

import pytest

from secrets_file import SecretsError, ensure_secrets

EXAMPLE = Path(__file__).resolve().parents[1] / "include" / "secrets.example.h"


def test_the_first_build_copies_the_example(tmp_path):
    secrets = tmp_path / "secrets.h"
    notice = ensure_secrets(EXAMPLE, secrets)
    assert secrets.read_text() == EXAMPLE.read_text()
    assert notice and "ignored by git" in notice


def test_an_existing_file_is_left_as_written(tmp_path):
    secrets = tmp_path / "secrets.h"
    text = EXAMPLE.read_text().replace('SECRET_WIFI_SSID ""', 'SECRET_WIFI_SSID "home"')
    secrets.write_text(text)
    assert ensure_secrets(EXAMPLE, secrets) is None
    assert secrets.read_text() == text


def test_a_define_the_example_has_must_be_present(tmp_path):
    secrets = tmp_path / "secrets.h"
    secrets.write_text('#define SECRET_WIFI_SSID ""\n#define SECRET_WIFI_PASSWORD ""\n')
    with pytest.raises(SecretsError, match="SECRET_AP_PASSWORD"):
        ensure_secrets(EXAMPLE, secrets)


@pytest.mark.parametrize("password", ["short", "x" * 65])
def test_an_access_point_password_wpa2_refuses_stops_the_build(tmp_path, password):
    secrets = tmp_path / "secrets.h"
    secrets.write_text(EXAMPLE.read_text().replace('"hexapod1"', f'"{password}"'))
    with pytest.raises(SecretsError, match="8 to 64"):
        ensure_secrets(EXAMPLE, secrets)
