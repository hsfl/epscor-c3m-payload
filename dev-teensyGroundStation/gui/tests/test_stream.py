import io
import sys
import unittest
from unittest.mock import Mock
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import gds_app as gui
from PIL import Image


def frame(seq=7, corrupt=False):
    pixels = bytes(range(256)) * 31 + bytes(range(64))
    checksum = ((sum(pixels) & 0xffff) + int(corrupt)) & 0xffff
    return b'WRM!' + bytes([seq, gui.gs_cli.STREAM_PACKETS_PER_FRAME]) + checksum.to_bytes(2, 'little') + pixels


class StreamTests(unittest.TestCase):
    def setUp(self):
        self.session = gui.GroundStationSession()
        self.session.stream_state = 'waiting'
        self.lines = []
        self.session._handle_line = self.lines.append

    def test_fragmented_frames_and_text(self):
        data = b'before\r\n' + frame() + b'after\n' + frame(8)
        for byte in data:
            self.session._process_bytes(bytes([byte]))
        self.assertEqual(self.lines, ['before', 'after'])
        self.assertEqual(self.session.last_stream['seq'], 8)
        self.assertEqual(self.session.stream_state, 'live')
        image = Image.open(io.BytesIO(self.session.last_stream['png']))
        self.assertEqual(image.size, (100, 80))
        self.assertEqual(image.mode, 'RGB')
        pixels = gui.np.frombuffer(frame()[8:], dtype=gui.np.uint8).reshape(80, 100)
        expected = gui.matplotlib.colormaps['hot'](pixels / 255.0, bytes=True)[:, :, :3]
        self.assertEqual(image.tobytes(), expected.tobytes())

    def test_corruption_then_recovery(self):
        self.session._process_bytes(frame(corrupt=True))
        self.assertIsNone(self.session.last_stream)
        self.session._process_bytes(frame(9))
        self.assertEqual(self.session.last_stream['seq'], 9)
        self.assertEqual(self.lines, [])

    def test_stopped_ignores_late_frames(self):
        self.session.stream_state = 'stopped'
        self.session._process_bytes(frame())
        self.assertIsNone(self.session.last_stream)
        self.assertEqual(self.session.stream_state, 'stopped')

    def test_commands_and_status(self):
        self.session.ser = Mock(is_open=True)
        self.session.send_command('stream start')
        self.session.ser.write.assert_called_with(b'stream start\n')
        self.assertEqual(self.session.stream_state, 'waiting')
        self.session._process_bytes(frame())
        self.session.send_command('stream stop')
        self.assertEqual(self.session.stream_state, 'stopping')
        gui.GroundStationSession._handle_line(self.session, 'Stream stop command sent (5x)')
        self.assertEqual(self.session.stream_state, 'stopping')
        self.assertFalse(self.session.stream_can_close)
        gui.GroundStationSession._handle_line(self.session, 'SAT> RPI STATUS: IDLE')
        self.assertEqual(self.session.stream_state, 'stopped')
        self.assertTrue(self.session.stream_can_close)
        self.session.send_command('stream start')
        self.assertFalse(self.session.stream_can_close)
        gui.GroundStationSession._handle_line(self.session, 'SAT> RPI STATUS: IDLE')
        self.assertFalse(self.session.stream_can_close)
        gui.GroundStationSession._handle_line(self.session, 'Failed to send stream start command')
        self.assertEqual(self.session.stream_state, 'error')

    def test_temperature_range(self):
        self.assertEqual(gui.temperature_range(['-5.25,20', 'nan,42.75']),
                         {"min": -5.25, "max": 42.75})
        self.assertIsNone(gui.temperature_range(['nan,nan']))

    def test_thermal_image_has_no_plot_margins(self):
        entry = {"rows": ["10,20,30", "40,50,60"]}
        image = Image.open(io.BytesIO(gui._render_thermal_png(entry, 'lepton', 0)))
        self.assertEqual(image.size, (3, 2))
        self.assertEqual(image.mode, 'RGB')
        rotated = Image.open(io.BytesIO(gui._render_thermal_png(entry, 'lepton', 90)))
        self.assertEqual(rotated.size, (2, 3))
        self.assertEqual(rotated.tobytes(), image.rotate(-90, expand=True).tobytes())

    def test_image_endpoint_and_state(self):
        original = gui.session
        gui.session = self.session
        try:
            client = gui.app.test_client()
            self.assertEqual(client.get('/api/stream/image').status_code, 404)
            self.session._process_bytes(frame())
            response = client.get('/api/stream/image')
            self.assertEqual(response.mimetype, 'image/png')
            self.assertEqual(response.headers['Cache-Control'], 'no-store')
            rotated = client.get('/api/stream/image?rotation=90')
            image = Image.open(io.BytesIO(rotated.data))
            original_image = Image.open(io.BytesIO(response.data))
            self.assertEqual(image.size, (80, 100))
            self.assertEqual(image.tobytes(), original_image.rotate(-90, expand=True).tobytes())
            self.assertEqual(client.get('/api/stream/image?rotation=45').status_code, 400)
            state = client.get('/api/state').get_json()
            self.assertEqual(state['stream_state'], 'live')
            self.assertEqual(state['stream_frame']['seq'], 7)
        finally:
            gui.session = original


if __name__ == '__main__':
    unittest.main()
