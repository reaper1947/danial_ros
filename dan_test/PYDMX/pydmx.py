
import serial
import time

class PyDMX:
    def __init__(self,COM='COM8',Cnumber=512,Brate=250000,Bsize=8,StopB=2):
        #start serial
        self.channel_num = Cnumber
        self.ser = serial.Serial(COM,baudrate=Brate,bytesize=Bsize,stopbits=StopB)
        self.data = [0]*self.channel_num
        self.data[0] = 0 # StartCode
        self.sleepms = 50.0
        self.breakus = 176.0
        self.MABus = 16.0


    def set_data(self,id,data):
        self.data[id]=data

    def set_datalist(self,list_id,list_data):
        try:
            for id,data in zip(list_id,list_data):
                self.set_data(id,data)
        except:
            print('list of id and data must be the same size!')

    def send(self):
        # Send Break : 88us - 1s
        self.ser.break_condition = True
        time.sleep(self.breakus/1000000.0)
        
        # Send MAB : 8us - 1s
        self.ser.break_condition = False
        time.sleep(self.MABus/1000000.0)
        
        # Send Data
        self.ser.write(bytearray(self.data))
        
        # Sleep
        time.sleep(self.sleepms/1000.0) # between 0 - 1 sec

    def sendzero(self):

        self.data = [0]*self.channel_num
        self.send()

    def __del__(self):
        print('Close serial server!')
        # self.sendzero()
        self.ser.close()
    
    def fade(self, start_channel, end_channel, start_intensity, end_intensity, duration, steps=50):
        """
        Gradually fades the intensity of a range of channels.
        
        Parameters:
            start_channel (int): The first DMX channel to fade.
            end_channel (int): The last DMX channel to fade.
            start_intensity (int): The starting intensity (0-255).
            end_intensity (int): The ending intensity (0-255).
            duration (float): Total duration of the fade in seconds.
            steps (int): Number of steps to divide the fade into.
        """
        if start_channel < 1 or end_channel > self.channel_num or start_channel > end_channel:
            raise ValueError("Invalid channel range.")

        step_delay = duration / steps
        intensity_step = (end_intensity - start_intensity) / steps

        for step in range(steps + 1):
            current_intensity = int(start_intensity + step * intensity_step)
            for channel in range(start_channel, end_channel + 1):
                self.set_data(channel, current_intensity)
            self.send()
            time.sleep(step_delay)
    
    def fade_hex(self, start_color, end_color, duration, steps=50):
        """
        Gradually fades between two colors specified by hex codes.
        
        Parameters:
            start_color (str): The starting color in hex format (e.g., '#FF5733').
            end_color (str): The ending color in hex format (e.g., '#33FF57').
            duration (float): Total duration of the fade in seconds.
            steps (int): Number of steps to divide the fade into.
        """
        # Convert hex colors to RGB components
        def hex_to_rgb(hex_color):
            hex_color = hex_color.lstrip('#')
            return tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))
        
        start_rgb = hex_to_rgb(start_color)
        end_rgb = hex_to_rgb(end_color)
        
        # Calculate step increments for each color channel
        step_delay = duration / steps
        r_step = (end_rgb[0] - start_rgb[0]) / steps
        g_step = (end_rgb[1] - start_rgb[1]) / steps
        b_step = (end_rgb[2] - start_rgb[2]) / steps

        for step in range(steps + 1):
            # Calculate current color intensities
            current_r = int(start_rgb[0] + step * r_step)
            current_g = int(start_rgb[1] + step * g_step)
            current_b = int(start_rgb[2] + step * b_step)
            
            # Set all fixtures to the current color
            for i in range(1, self.channel_num, 4):
                if i < self.channel_num:
                    self.set_data(i, current_r)     # Red channel
                if i + 2 < self.channel_num:
                    self.set_data(i + 2, current_g) # Green channel
                if i + 3 < self.channel_num:
                    self.set_data(i + 3, current_b) # Blue channel
            
            self.send()  # Send the data to DMX
            time.sleep(step_delay)  # Pause for the step duration

    def fade_hex_fast(self, start_color, end_color, speed=1.0, steps=50):
        """
        Quickly fades between two colors specified by hex codes.
        
        Parameters:
            start_color (str): The starting color in hex format (e.g., '#FF5733').
            end_color (str): The ending color in hex format (e.g., '#33FF57').
            speed (float): Speed factor. Higher values make the fade faster.
            steps (int): Number of steps to divide the fade into.
        """
        # Convert hex colors to RGB components
        def hex_to_rgb(hex_color):
            hex_color = hex_color.lstrip('#')
            return tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))
        
        start_rgb = hex_to_rgb(start_color)
        end_rgb = hex_to_rgb(end_color)
        
        # Calculate step increments for each color channel
        step_delay = max(0.01, (1.0 / speed) / steps)  # Min delay ensures smooth performance
        r_step = (end_rgb[0] - start_rgb[0]) / steps
        g_step = (end_rgb[1] - start_rgb[1]) / steps
        b_step = (end_rgb[2] - start_rgb[2]) / steps

        for step in range(steps + 1):
            # Calculate current color intensities
            current_r = int(start_rgb[0] + step * r_step)
            current_g = int(start_rgb[1] + step * g_step)
            current_b = int(start_rgb[2] + step * b_step)
            
            # Set all fixtures to the current color
            for i in range(1, self.channel_num, 4):
                if i < self.channel_num:
                    self.set_data(i, current_r)     # Red channel
                if i + 2 < self.channel_num:
                    self.set_data(i + 2, current_g) # Green channel
                if i + 3 < self.channel_num:
                    self.set_data(i + 3, current_b) # Blue channel
            
        self.send()  # Send the data to DMX
        time.sleep(step_delay)  # Pause for the step duration
        
    def fade_hex_time(self, start_color, end_color, duration=3):
        """
        Fades from the start color to the end color over a set duration.
        
        Parameters:
            start_color (str): The starting color in hex format (e.g., '#FF5733').
            end_color (str): The ending color in hex format (e.g., '#33FF57').
            duration (float): Total duration of the fade in seconds.
        """
        # Convert hex colors to RGB components
        def hex_to_rgb(hex_color):
            hex_color = hex_color.lstrip('#')
            return tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))
        
        start_rgb = hex_to_rgb(start_color)
        end_rgb = hex_to_rgb(end_color)
        
        # Calculate number of steps for smooth fade
        steps = int(duration * 30)  # 30 steps per second for smooth fading
        step_delay = duration / steps
        
        # Calculate step increments for each color channel
        r_step = (end_rgb[0] - start_rgb[0]) / steps
        g_step = (end_rgb[1] - start_rgb[1]) / steps
        b_step = (end_rgb[2] - start_rgb[2]) / steps

        for step in range(steps + 1):
            # Calculate current color intensities
            current_r = int(start_rgb[0] + step * r_step)
            current_g = int(start_rgb[1] + step * g_step)
            current_b = int(start_rgb[2] + step * b_step)
            
            # Set all fixtures to the current color
            for i in range(1, self.channel_num, 4):
                if i < self.channel_num:
                    self.set_data(i, current_r)     # Red channel
                if i + 2 < self.channel_num:
                    self.set_data(i + 2, current_g) # Green channel
                if i + 3 < self.channel_num:
                    self.set_data(i + 3, current_b) # Blue channel
            
            self.send()  # Send the data to DMX
            time.sleep(step_delay)  # Pause for the step duration
    
    def set_color(self, hex_color):
        """
        Sets the color of the entire LED strip using a hex color code.
        
        Parameters:
            hex_color (str): The color in hex format (e.g., '#FF5733').
        """
        def hex_to_rgb(hex_color):
            hex_color = hex_color.lstrip('#')
            # Convert the hex to RGB components
            return tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))

        start_rgb = hex_to_rgb(hex_color)
        
        # Assuming the first 3 channels are for RGB and the 4th channel is for White or intensity (if applicable)
        for i in range(1, self.channel_num, 4):
            # Set RGB values for each LED
            if i < self.channel_num:
                self.set_data(i, start_rgb[0])  # Red channel
            if i + 1 < self.channel_num:
                self.set_data(i + 1, start_rgb[1])  # Green channel
            if i + 2 < self.channel_num:
                self.set_data(i + 2, start_rgb[2])  # Blue channel
            if i + 3 < self.channel_num:
                self.set_data(i + 3, 0)  # White channel (set to 0, or adjust if needed)

        # Send data to DMX
        self.send()