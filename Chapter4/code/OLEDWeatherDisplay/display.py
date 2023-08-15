import Adafruit_SSD1306
from PIL import Image, ImageDraw, ImageFont
from WeatherData import WeatherData

disp = Adafruit_SSD1306.SSD1306_128_32(rst=None)
disp.begin()

# Create blank image for drawing.
width = disp.width
height = disp.height
image = Image.new('1', (width, height))
draw = ImageDraw.Draw(image)
font = ImageFont.load_default()

city = 'Toronto'
weather = WeatherData(city)
temperature = weather.get_temperature()
conditions = weather.get_conditions()

# Draw the weather data
draw.text((0, 0), 'City: ' + city, font=font, fill=255)
draw.text((0, 10), 'Temp: ' + temperature + '°C', font=font, fill=255)
draw.text((0, 20), 'Cond: ' + conditions, font=font, fill=255)

# Display image
disp.image(image)
disp.display()
