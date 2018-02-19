using System;
using System.Threading;
using ArduinoUploader.BootloaderProgrammers.Protocols.STK500v1.Messages;
using ArduinoUploader.Hardware;
using ArduinoUploader.Hardware.Memory;

namespace ArduinoUploader.BootloaderProgrammers.Protocols.STK500v1
{
    internal class Stk500V1BootloaderProgrammer : ArduinoBootloaderProgrammer
    {
        internal Stk500V1BootloaderProgrammer(SerialPortConfig serialPortConfig, IMcu mcu)
            : base(serialPortConfig, mcu)
        {
        }

        public override void EstablishSync()
        {
            Send(new GetSyncRequest());

            const int maxRetries = 256;
            var retryCounter = 0;
            while (retryCounter++ < maxRetries)
            {
                var result = Receive<GetSyncResponse>();
                if (result == null)
                {
                    Send(new GetSyncRequest());
                    continue;
                }
                if (result.IsInSync) break;
                Thread.Sleep(20);
            }
            if (retryCounter == maxRetries)
                throw new ArduinoUploaderException(
                    $"Unable to establish sync after {maxRetries} retries.");

            retryCounter = 0;
            while (retryCounter++ < maxRetries)
            {
                var nextByte = ReceiveNext();
                if (nextByte == Constants.RespStkOk) break;
            }
            if (retryCounter == maxRetries)
                throw new ArduinoUploaderException("Unable to establish sync.");
        }

        protected void SendWithSyncRetry(IRequest request)
        {
            var nextByte = new byte[1];
            int cntr = 0;
            while (true)
            {
                
                Send(request);
                //Thread.Sleep(500);
                nextByte = ReceiveNext(1);
                if(nextByte[0] == Constants.RespStkInsync || nextByte[0] == Constants.RespStkOk || request.GetType() == typeof(GetParameterRequest) || request.GetType() == typeof(SetDeviceProgrammingParametersRequest) || request.GetType() == typeof(EnableProgrammingModeRequest))
                {
                    //EstablishSync();
                    break;
                }

                
            }
            if ((nextByte[0] != Constants.RespStkInsync && nextByte[0] != Constants.RespStkOk) && request.GetType() != typeof(GetParameterRequest) && request.GetType() != typeof(SetDeviceProgrammingParametersRequest) && request.GetType() != typeof(EnableProgrammingModeRequest))
            {
                throw new ArduinoUploaderException(
                    $"Unable to aqcuire sync in SendWithSyncRetry for request of type {request.GetType()}!");
            }
            //else
            //{
            //    if (nextByte[0] == Constants.RespStkOk || nextByte[0] == Constants.RespStkInsync)
            //    {
            //        nextByte = ReceiveNext(1);
            //    }
            //}

        }

        public override void CheckDeviceSignature()
        {
            
            Logger?.Debug($"Expecting to find '{Mcu.DeviceSignature}'...");
            Thread.Sleep(1000);
            SendWithSyncRetry(new ReadSignatureRequest());
            //bytes[0] = (byte)'u';
            //SerialPort.Write(bytes, 1, ref bytesWritten);
            //Thread.Sleep(250);
            //bytes[0] = Constants.SyncCrcEop;
            //SerialPort.Write(bytes, 1, ref bytesWritten);
            Thread.Sleep(1000);
            var response = Receive<ReadSignatureResponse>(3);
            //var response = ReceiveNext();
            if (response == null )
                throw new ArduinoUploaderException("Unable to check device signature!");

            ////var signature = response.Signature;
            //if (BitConverter.ToString(signature) != Mcu.DeviceSignature)
            //    throw new ArduinoUploaderException(
            //        $"Unexpected device signature - found '{BitConverter.ToString(signature)}'- expected '{Mcu.DeviceSignature}'.");
        }

        public override void InitializeDevice()
        {
            var majorVersion = GetParameterValue(Constants.ParmStkSwMajor);
            var minorVersion = GetParameterValue(Constants.ParmStkSwMinor);
            Logger?.Info($"Retrieved software version: {majorVersion}.{minorVersion}.");

            Logger?.Info("Setting device programming parameters...");
            SendWithSyncRetry(new SetDeviceProgrammingParametersRequest((Mcu) Mcu));
            var nextByte = ReceiveNext();

            if (nextByte != Constants.RespStkOk && nextByte != Constants.RespStkInsync)
                throw new ArduinoUploaderException("Unable to set device programming parameters!");
        }

        public override void EnableProgrammingMode()
        {
            SendWithSyncRetry(new EnableProgrammingModeRequest());
            var nextByte = ReceiveNext();
            if (nextByte == Constants.RespStkOk || nextByte == Constants.RespStkInsync) return;
            if (nextByte == Constants.RespStkNodevice || nextByte == Constants.RespStkFailed)
                throw new ArduinoUploaderException("Unable to enable programming mode on the device!");
        }

        public override void LeaveProgrammingMode()
        {
            SendWithSyncRetry(new LeaveProgrammingModeRequest());
            var nextByte = ReceiveNext();
            if (nextByte == Constants.RespStkOk || nextByte == Constants.RespStkInsync) return;
            if (nextByte == Constants.RespStkNodevice || nextByte == Constants.RespStkFailed)
                throw new ArduinoUploaderException("Unable to leave programming mode on the device!");
        }

        private uint GetParameterValue(byte param)
        {
            Logger?.Trace($"Retrieving parameter '{param}'...");
            SendWithSyncRetry(new GetParameterRequest(param));
            var nextByte = ReceiveNext();
            var paramValue = (uint) nextByte;
            nextByte = ReceiveNext();

            //if (nextByte == Constants.RespStkFailed)
            //    throw new ArduinoUploaderException($"Retrieving parameter '{param}' failed!");

            //if (nextByte != Constants.RespStkOk && nextByte != 0x03)
            //    throw new ArduinoUploaderException(
            //        $"General protocol error while retrieving parameter '{param}'.");

            return paramValue;
        }

        public override void ExecuteWritePage(IMemory memory, int offset, byte[] bytes)
        {
            SendWithSyncRetry(new ExecuteProgramPageRequest(memory, bytes));
            //if (nextByte == Constants.RespStkOk) return;
            //throw new ArduinoUploaderException($"Write at offset {offset} failed!");
            return;
        }

        public override byte[] ExecuteReadPage(IMemory memory)
        {
            var pageSize = memory.PageSize;
            SendWithSyncRetry(new ExecuteReadPageRequest(memory.Type, pageSize));
            var bytes = ReceiveNext(pageSize);
            if (bytes == null)
                throw new ArduinoUploaderException("Execute read page failed!");

            var nextByte = ReceiveNext();
            //if (nextByte == Constants.RespStkOk) return bytes;
            //throw new ArduinoUploaderException("Execute read page failed!");
            return bytes;
        }

        public override void LoadAddress(IMemory memory, int addr)
        {
            Logger?.Trace($"Sending load address request: {addr}.");
            addr = addr >> 1;
            SendWithSyncRetry(new LoadAddressRequest(addr));
            var result = ReceiveNext();
            //if (result == Constants.RespStkOk) return;
            //throw new ArduinoUploaderException($"LoadAddress failed with result {result}!");
            return;
        }
    }
}