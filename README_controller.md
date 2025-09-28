# ESP32 Slider Controller - Flask Web Application

A beautiful web interface to control your ESP32 slider via OSC messages.

## Features

- 🎮 **Real-time Jog Control**: Smooth slider control with speed adjustment
- ↔️ **Pan/Tilt Offsets**: Fine manual adjustments for pan and tilt
- 📍 **Preset Management**: Set and manage position presets
- 🎨 **Modern UI**: Beautiful, responsive web interface
- 📱 **Mobile Friendly**: Works on desktop, tablet, and mobile devices

## OSC Commands Supported

The web interface sends the following OSC messages to your ESP32:

- `/jog` (float): Movement speed from -1.0 to 1.0
- `/pan` (float): Pan offset from -1.0 to 1.0  
- `/tilt` (float): Tilt offset from -1.0 to 1.0
- `/setPresetA` (int, int, int, int): Set Preset A (pan, tilt, zoom, slide in steps)
- `/setPresetB` (int, int, int, int): Set Preset B (pan, tilt, zoom, slide in steps)

## Setup Instructions

### 1. Install Python Dependencies

```bash
pip install -r requirements.txt
```

### 2. Configure ESP32 IP Address

Edit `config.py` and update the ESP32 IP address:

```python
ESP32_IP = "192.168.1.100"  # Change to your ESP32's IP
```

### 3. Run the Web Server

```bash
python slider_controller.py
```

### 4. Access the Web Interface

Open your browser and go to: `http://localhost:5000`

## Usage

### Jog Control
- Use the **Jog Control** slider to move the slider between presets
- Range: -1.0 (reverse) to 1.0 (forward)
- Click **Stop** to halt all movement

### Manual Adjustments
- **Pan Control**: Fine-tune horizontal position
- **Tilt Control**: Fine-tune vertical position
- Use **Reset** buttons to return to center

### Preset Management
- Set **Preset A** and **Preset B** with specific step values
- Use jog control to smoothly move between presets
- Presets are stored in steps (motor steps)

## Network Requirements

- ESP32 and computer must be on the same network
- ESP32 must be running the slider firmware
- ESP32 OSC server must be listening on port 8000

## Troubleshooting

### Connection Issues
1. Check that ESP32 IP address is correct in `config.py`
2. Verify ESP32 is connected to WiFi
3. Ensure ESP32 slider firmware is running
4. Check firewall settings

### OSC Messages Not Working
1. Verify ESP32 is receiving OSC messages (check serial output)
2. Check network connectivity between devices
3. Ensure OSC port 8000 is not blocked

## File Structure

```
slider_controller.py    # Main Flask application
templates/
  index.html           # Web interface template
config.py              # Configuration settings
requirements.txt       # Python dependencies
README_controller.md   # This file
```

## Customization

### Adding New OSC Commands
1. Add new route in `slider_controller.py`
2. Update the web interface in `templates/index.html`
3. Add corresponding JavaScript function

### Styling Changes
- Modify CSS in the `<style>` section of `index.html`
- Colors, fonts, and layout can be customized

### OSC Message Format
The application sends OSC messages in the format:
- Address: `/command`
- Arguments: Type-tagged values (float, int)

Example OSC message structure:
```
/jog,0.5    # Send jog command with value 0.5
/pan,0.2    # Send pan offset with value 0.2
```

## License

This project is open source. Feel free to modify and distribute.

