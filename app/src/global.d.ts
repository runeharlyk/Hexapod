export {}

declare global {
  interface Navigator {
    bluetooth: {
      requestDevice(options: {
        acceptAllDevices: boolean
        optionalServices: string[]
      }): Promise<BluetoothDevice>
    }
  }

  interface BluetoothDevice {
    gatt?: {
      connect(): Promise<BluetoothRemoteGATTServer>
      connected: boolean
      disconnect(): Promise<void>
    }
  }

  interface BluetoothRemoteGATTServer {
    getPrimaryService(service: string): Promise<BluetoothRemoteGATTService>
  }

  interface BluetoothRemoteGATTService {
    getCharacteristic(characteristic: string): Promise<BluetoothRemoteGATTCharacteristic>
  }

  interface BluetoothRemoteGATTCharacteristic {
    startNotifications(): Promise<BluetoothRemoteGATTCharacteristic>
    writeValue(value: BufferSource): Promise<void>
    addEventListener(
      type: string,
      listener: (event: { target: { value: { buffer: ArrayBuffer } } }) => void
    ): void
  }
}
