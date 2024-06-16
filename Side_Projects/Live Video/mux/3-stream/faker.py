video1 = "./inputs/vid1.av1"
video2 = "./inputs/vid2.av1"

output_dest = "./output/mux.bin"

source1_size = 1250
source2_size = 1250

telemetry_string = "Source:KC3TM,Destination:321,Path:555,Type:Position Without Timestamp,Data:!7643.21N/3612.34W[359/050/A=001234/S1/002/180/359/010,RSSI:-120"

# Read the video files and write out a text file of the two videos and a constant string multiplexed together
# The output file will be used as input to the demuxer
# The format is the number 1, followed by the size of source 1 in the next 2 bytes, followed by the specific bytes of source 1.
# The same is done for source 2. This is repeated 15 times, alternating between the two sources. Then, the number 3 is written, 
# followed by the size of the constant string, followed by the constant string. This pattern is repeated until the end of the file.
def write_mux():
    with open(video1, "rb") as f1, open(video2, "rb") as f2, open(output_dest, "wb") as out:
        # repeat while there are still bytes to read from the files
        escape = False
        while not escape:
            for i in range(50):
                # read source 1
                source1 = f1.read(source1_size)
                # read source 2
                source2 = f2.read(source2_size)
                # if either source is empty, break the loop
                if not source1 and not source2:
                    escape = True
                # write source 1
                out.write(b'\x01')
                out.write(source1_size.to_bytes(2, byteorder='big'))
                out.write(source1)
                # write source 2
                out.write(b'\x02')
                out.write(source2_size.to_bytes(2, byteorder='big'))
                out.write(source2)
            # out.write(b'\xfe')
            # out.write(len(telemetry_string).to_bytes(2, byteorder='big'))
            # out.write(telemetry_string.encode('utf-8', 'strict'))

if __name__ == "__main__":
    write_mux()