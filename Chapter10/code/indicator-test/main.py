from indicator import Indicator
import utime

# Example usage
indicator = Indicator()

# Test the indicator
indicator.set_indicator(1)   # Should be predominantly red
utime.sleep(1)
indicator.set_indicator(90)  # Should be predominantly blue
utime.sleep(1)
indicator.set_indicator(50)  # Should be predominantly green