"""Combine the rpicam + Lepton viewer PNGs into one keepsake photo.

Pure PIL compositing - no Flask/session dependency, so it's easy to test
standalone with any two PNGs.
"""

import io
import os

from PIL import Image, ImageDraw, ImageFont

DEFAULT_LOGO_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static", "logos")
HSFL_LOGO_PATH = os.path.join(DEFAULT_LOGO_DIR, "hsfl_logo.png")
C3M_LOGO_PATH = os.path.join(DEFAULT_LOGO_DIR, "c3m_logo.png")

TARGET_PANEL_HEIGHT = 480  # px - both panels (and the colorbar, if shown) are resized to this height
GUTTER_PX = 12  # gap between panels (and between the lepton panel and its colorbar, if shown)
MARGIN_PX = 16  # canvas edge margin

HEADER_TEXT = "Hawaii Space Flight Laboratory, SmallSat 2026"
HEADER_SUBTITLE_TEXT = "C3M: CubeSats for Climate Change Monitoring"
HEADER_LOGO_HEIGHT = 70  # px - logos flank the header text, above the rpicam image
HEADER_FONT_SIZE = 26
HEADER_SUBTITLE_FONT_SIZE = 16
HEADER_LINE_GAP_PX = 4  # gap between the title and subtitle lines
HEADER_GAP_PX = 12  # gap between the header row and the image panels below it
TEXT_COLOR = (30, 30, 30)
SUBTITLE_COLOR = (90, 90, 90)

BACKGROUND = (255, 255, 255)  # this is a shareable keepsake photo, not a dark UI panel


def _resize_to_height(img, height):
    w, h = img.size
    scale = height / h
    return img.resize((max(1, round(w * scale)), height), Image.LANCZOS)


def _load_logo(logo_path, height):
    """Return the logo resized to the given height (RGBA), or None if it isn't there yet."""
    if not os.path.exists(logo_path):
        return None
    logo = Image.open(logo_path).convert("RGBA")
    if logo.height != height:
        logo = _resize_to_height(logo, height)
    return logo


def compose_visitor_image(
    rpicam_png_bytes,
    lepton_png_bytes,
    colorbar_png_bytes=None,
    hsfl_logo_path=HSFL_LOGO_PATH,
    c3m_logo_path=C3M_LOGO_PATH,
):
    """Combine rpicam + Lepton PNGs side by side, with a header row (logos + event
    text) above the rpicam image. If colorbar_png_bytes is given, it's appended
    as a strip after the lepton panel, scaled to the panel's exact height.

    Returns PNG bytes.
    """
    rpicam_img = _resize_to_height(Image.open(io.BytesIO(rpicam_png_bytes)).convert("RGB"), TARGET_PANEL_HEIGHT)
    lepton_img = _resize_to_height(Image.open(io.BytesIO(lepton_png_bytes)).convert("RGB"), TARGET_PANEL_HEIGHT)

    colorbar_img = None
    if colorbar_png_bytes is not None:
        colorbar_img = _resize_to_height(Image.open(io.BytesIO(colorbar_png_bytes)).convert("RGB"), TARGET_PANEL_HEIGHT)

    canvas_w = MARGIN_PX * 2 + rpicam_img.width + GUTTER_PX + lepton_img.width
    if colorbar_img is not None:
        canvas_w += GUTTER_PX + colorbar_img.width

    font = ImageFont.load_default(size=HEADER_FONT_SIZE)
    subtitle_font = ImageFont.load_default(size=HEADER_SUBTITLE_FONT_SIZE)
    measure = ImageDraw.Draw(Image.new("RGB", (1, 1)))
    title_bbox = measure.textbbox((0, 0), HEADER_TEXT, font=font)
    subtitle_bbox = measure.textbbox((0, 0), HEADER_SUBTITLE_TEXT, font=subtitle_font)
    title_h = int(title_bbox[3] - title_bbox[1])
    subtitle_h = int(subtitle_bbox[3] - subtitle_bbox[1])
    text_block_h = title_h + HEADER_LINE_GAP_PX + subtitle_h

    header_h = max(HEADER_LOGO_HEIGHT, text_block_h)
    canvas_h = MARGIN_PX + header_h + HEADER_GAP_PX + TARGET_PANEL_HEIGHT + MARGIN_PX
    canvas = Image.new("RGB", (canvas_w, canvas_h), BACKGROUND)
    draw = ImageDraw.Draw(canvas)

    # Header row: hsfl logo on the left, c3m logo on the right, title +
    # subtitle text centered between them as one block.
    header_center_y = MARGIN_PX + header_h // 2
    text_top = header_center_y - text_block_h // 2

    title_x = (canvas_w - (title_bbox[2] - title_bbox[0])) // 2 - title_bbox[0]
    title_y = text_top - title_bbox[1]
    draw.text((title_x, title_y), HEADER_TEXT, font=font, fill=TEXT_COLOR)

    subtitle_x = (canvas_w - (subtitle_bbox[2] - subtitle_bbox[0])) // 2 - subtitle_bbox[0]
    subtitle_y = text_top + title_h + HEADER_LINE_GAP_PX - subtitle_bbox[1]
    draw.text((subtitle_x, subtitle_y), HEADER_SUBTITLE_TEXT, font=subtitle_font, fill=SUBTITLE_COLOR)

    hsfl_logo = _load_logo(hsfl_logo_path, HEADER_LOGO_HEIGHT)
    if hsfl_logo is not None:
        canvas.paste(hsfl_logo, (MARGIN_PX, header_center_y - hsfl_logo.height // 2), hsfl_logo)

    c3m_logo = _load_logo(c3m_logo_path, HEADER_LOGO_HEIGHT)
    if c3m_logo is not None:
        c3m_x = canvas_w - MARGIN_PX - c3m_logo.width
        canvas.paste(c3m_logo, (c3m_x, header_center_y - c3m_logo.height // 2), c3m_logo)

    images_y = MARGIN_PX + header_h + HEADER_GAP_PX
    canvas.paste(rpicam_img, (MARGIN_PX, images_y))
    lepton_x = MARGIN_PX + rpicam_img.width + GUTTER_PX
    canvas.paste(lepton_img, (lepton_x, images_y))
    if colorbar_img is not None:
        canvas.paste(colorbar_img, (lepton_x + lepton_img.width + GUTTER_PX, images_y))

    buf = io.BytesIO()
    canvas.save(buf, format="PNG")
    buf.seek(0)
    return buf.getvalue()


def logo_status():
    return {"hsfl": os.path.exists(HSFL_LOGO_PATH), "c3m": os.path.exists(C3M_LOGO_PATH)}
