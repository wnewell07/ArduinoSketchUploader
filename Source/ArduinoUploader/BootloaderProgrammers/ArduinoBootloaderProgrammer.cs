using System;
using System.Threading;
using System.Linq;

using ArduinoUploader.BootloaderProgrammers.Protocols;
using ArduinoUploader.Hardware;

using FTD2XX_NET;
using ArduinoUploader.BootloaderProgrammers.Protocols.STK500v1;
//using System.IO.Ports;
//using RJCP.IO.Ports;

namespace ArduinoUploader.BootloaderProgrammers {

	internal abstract class ArduinoBootloaderProgrammer : BootloaderProgrammer {

		private FTDI.FT_STATUS _status = FTDI.FT_STATUS.FT_OK;
		protected SerialPortConfig SerialPortConfig;

		protected ArduinoBootloaderProgrammer(SerialPortConfig serialPortConfig, IMcu mcu)
			: base(mcu) {
			SerialPortConfig = serialPortConfig;
		}

		protected FTDI SerialPort { get; set; }

		public override void Open() {
			var portName = SerialPortConfig.PortName;
			var baudRate = SerialPortConfig.BaudRate;

			Logger?.Info($"Opening serial port {portName} - baudrate {baudRate}");

			var description = "NEWGY3050";

			// Create new instance of the FTDI device class
			SerialPort?.Close();
			SerialPort = new FTDI();

			// Open first device in our list by the description
			_status = SerialPort.OpenByDescription(description);

			// Reset the device
			_status = SerialPort.ResetDevice();

			// Purge the device
			_status = SerialPort.Purge(3);

			// Set the baud rate to 115200 
			_status = SerialPort.SetBaudRate(115200);

			// Set data characteristics - Data bits, Stop bits, Parity
			_status = SerialPort.SetDataCharacteristics(FTDI.FT_DATA_BITS.FT_BITS_8, FTDI.FT_STOP_BITS.FT_STOP_BITS_1, FTDI.FT_PARITY.FT_PARITY_NONE);

			// Set read timeout to 1 seconds, write timeout to 100
			_status = SerialPort.SetTimeouts(1, 100);
           // _status = SerialPort.

			// Set flow control - set RTS/CTS flow control
			_status = SerialPort.SetFlowControl(FTDI.FT_FLOW_CONTROL.FT_FLOW_NONE, 0, 0);

			// Set RTS
			_status = SerialPort.SetRTS(true);

			// Set DTR
			_status = SerialPort.SetDTR(true);

			Logger?.Trace($"Opened serial port {portName} with baud rate {baudRate}!");

			var postOpen = SerialPortConfig.PostOpenResetBehavior;
			if (postOpen != null) {
				Logger?.Info($"Executing Post Open behavior ({postOpen})...");
				//SerialPort = postOpen.Reset(SerialPort, SerialPortConfig);
			}

			var sleepAfterOpen = SerialPortConfig.SleepAfterOpen;
			if (SerialPortConfig.SleepAfterOpen <= 0)
				return;

			Logger?.Trace($"Sleeping for {sleepAfterOpen} ms after open...");
			Thread.Sleep(sleepAfterOpen);
		}

        public void sendUpdateCommand()
        {
            uint bytesWritten = 0;
            var bytes = new byte[1];
            bytes[0] = (byte)'X';
            SerialPort.Write(bytes, 1, ref bytesWritten);
            Thread.Sleep(250);
            SerialPort.Write(bytes, 1, ref bytesWritten);
            Thread.Sleep(250);
        }

		public override void EstablishSync() {
			// Do nothing.
		}

		public override void Close() {
			var preClose = SerialPortConfig.CloseResetAction;
			if (preClose != null) {
				Logger?.Info("Resetting...");
				//SerialPort = preClose.Reset(SerialPort, SerialPortConfig);
			}

			Logger?.Info("Closing serial port...");
			try {
				SerialPort.Close();
			} catch (Exception) {
				// Ignore
			}
		}

		protected virtual void Send(IRequest request) {
			var bytes = request.Bytes;
			var length = bytes.Length;
            uint tmp = 0;
			Logger?.Debug($"Sending {length} bytes: {Environment.NewLine}"
			              + $"{BitConverter.ToString(bytes)}");
			UInt32 numBytesWritten = 0;
			SerialPort.Write(bytes, length, ref numBytesWritten);
            tmp = numBytesWritten;
		}

		protected TResponse Receive<TResponse>(int length = 1)
			where TResponse : Response, new() {
			var bytes = ReceiveNext(length);
			if (bytes == null)
				return null;
			return new TResponse {Bytes = bytes};
		}

		protected int ReceiveNext() {
			var bytes = new byte[1];
            
			try {
				UInt32 numBytesRead = 0;
				SerialPort.Read(bytes, 1, ref numBytesRead);
                if(numBytesRead == 0)
                {
                    throw new TimeoutException();
                }
				Logger?.Debug($"Receiving byte: {BitConverter.ToString(bytes)}");
				return bytes[0];
			} catch (TimeoutException) {
                //return null;
                return -1;
			}
		}

		protected byte[] ReceiveNext(int length) {
			var bytes = new byte[length];
			uint retrieved = 0;
            uint prevRetrieved = 0;
            DateTime timeout = DateTime.Now.AddSeconds(5);

			try {

				UInt32 numBytesRead = 0;
                while (retrieved < length)
                {
                    if (length > 120)
                    {
                        var singleByte = new byte[1];
                        SerialPort.Read(singleByte, 1, ref numBytesRead);
                        //SerialPort.Read(singleByte, 1, ref numBytesRead);
                    }
                    SerialPort.Read(bytes, (uint)length, ref numBytesRead);

                    retrieved += numBytesRead;

                    
                    Thread.Sleep(50);
                    if (DateTime.Now > timeout)
                    {
                        throw new TimeoutException();
                    }
                    
                    prevRetrieved = retrieved;
                }
            
                    Logger?.Debug($"Receiving bytes: {BitConverter.ToString(bytes)}");
                
				return bytes;
			} catch (TimeoutException) {

                return null;
			}
		}

	}

}