"""Email the combined visitor photo via SMTP.

Config is read from the environment (populated by python-dotenv's
load_dotenv() call in gds_app.py) per call rather than cached at import, so
editing .env only needs an app restart, not a code change, to take effect.
"""

import os
import smtplib
from email.message import EmailMessage

REQUIRED_ENV_VARS = ("SMTP_HOST", "SMTP_PORT", "SMTP_USERNAME", "SMTP_PASSWORD", "SMTP_FROM_EMAIL")


class OutreachConfigError(Exception):
    """Required SMTP config is missing from the environment/.env."""


class OutreachSendError(Exception):
    """The send was attempted but smtplib raised an error."""


def _get_config():
    missing = [name for name in REQUIRED_ENV_VARS if not os.environ.get(name)]
    if missing:
        raise OutreachConfigError(f"email is not configured - missing: {', '.join(missing)}")

    return {
        "host": os.environ["SMTP_HOST"],
        "port": int(os.environ["SMTP_PORT"]),
        "use_ssl": os.environ.get("SMTP_USE_SSL", "true").lower() not in ("0", "false", "no"),
        "username": os.environ["SMTP_USERNAME"],
        "password": os.environ["SMTP_PASSWORD"],
        "from_name": os.environ.get("SMTP_FROM_NAME", "EPSCOR C3M"),
        "from_email": os.environ["SMTP_FROM_EMAIL"],
    }


def email_configured():
    try:
        _get_config()
        return True
    except OutreachConfigError:
        return False


def send_email(to_email, subject, body, attachment_bytes, attachment_filename="photo.png"):
    config = _get_config()

    msg = EmailMessage()
    msg["Subject"] = subject
    msg["From"] = f"{config['from_name']} <{config['from_email']}>"
    msg["To"] = to_email
    msg.set_content(body)
    msg.add_attachment(attachment_bytes, maintype="image", subtype="png", filename=attachment_filename)

    try:
        if config["use_ssl"]:
            with smtplib.SMTP_SSL(config["host"], config["port"], timeout=20) as server:
                server.login(config["username"], config["password"])
                server.send_message(msg)
        else:
            with smtplib.SMTP(config["host"], config["port"], timeout=20) as server:
                server.starttls()
                server.login(config["username"], config["password"])
                server.send_message(msg)
    except (smtplib.SMTPException, OSError) as e:
        raise OutreachSendError(f"failed to send email: {e}") from e
