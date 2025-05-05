import { writable } from 'svelte/store';
import { notifications } from './components/toasts/notifications';

export const SERVICE_UUID = '6e400001-b5a3-f393-e0a9-e50e24dcca9e';
const CHARACTERISTIC_TX_UUID = '6e400003-b5a3-f393-e0a9-e50e24dcca9e';
const CHARACTERISTIC_RX_UUID = '6e400002-b5a3-f393-e0a9-e50e24dcca9e';

export const isConnected = writable<boolean>(false);
export const receivedData = writable<string>('');

let bluetoothDevice: BluetoothDevice | null = null;
let server: BluetoothRemoteGATTServer | null = null;
let txCharacteristic: BluetoothRemoteGATTCharacteristic | null = null;
let rxCharacteristic: BluetoothRemoteGATTCharacteristic | null = null;

export async function connect() {
    if (!navigator.bluetooth) {
        notifications.error('Bluetooth not supported');
        return;
    }

    console.log('Connecting to Bluetooth...');
    try {
        // const btPermission = await navigator.permissions.query({ name: 'bluetooth' });
        // if (btPermission.state === 'denied') {
        //     notifications.error('Bluetooth permission denied');
        //     return;
        // } else if (btPermission.state === 'prompt') {
        //     notifications.info('Bluetooth permission requested');
        // } else if (btPermission.state === 'granted') {
        //     notifications.info('Bluetooth permission granted');
        // }

        notifications.info('Requesting Bluetooth device...');
        bluetoothDevice = await navigator.bluetooth.requestDevice({
            // filters: [{ services: [SERVICE_UUID] }]
            // Optional: Use acceptAllDevices: true if you want to see all devices
            acceptAllDevices: true,
            optionalServices: [SERVICE_UUID] // Might be needed depending on browser implementation
        });

        if (!bluetoothDevice || !bluetoothDevice.gatt) {
            throw new Error('No device selected or GATT not available.');
        }

        notifications.info('Connecting to GATT server...');
        server = await bluetoothDevice.gatt.connect();
        notifications.info('Connected to GATT server');

        notifications.info('Getting primary service...');
        const service = await server.getPrimaryService(SERVICE_UUID);
        notifications.info('Got primary service');

        notifications.info('Getting TX characteristic...');
        txCharacteristic = await service.getCharacteristic(CHARACTERISTIC_TX_UUID);
        notifications.info('Got TX characteristic');

        notifications.info('Getting RX characteristic...');
        rxCharacteristic = await service.getCharacteristic(CHARACTERISTIC_RX_UUID);
        notifications.info('Got RX characteristic');

        notifications.info('Starting notifications...');
        await txCharacteristic.startNotifications();
        txCharacteristic.addEventListener('characteristicvaluechanged', handleNotifications);

        isConnected.set(true);
        bluetoothDevice.addEventListener('gattserverdisconnected', onDisconnected);
        notifications.info('Bluetooth connection established.');

        txCharacteristic.writeValueWithoutResponse(new TextEncoder().encode('Hello, world!'));
    } catch (error) {
        notifications.error('Bluetooth connection failed:' + error);
        console.error('Bluetooth connection failed:', error);
        isConnected.set(false);
        // Optionally notify the user via a store or event
    }
}

export async function disconnect() {
    console.log('Disconnecting from Bluetooth...');
    if (!bluetoothDevice) {
        notifications.error('No device to disconnect from.');
        return;
    }

    if (bluetoothDevice.gatt?.connected) {
        notifications.info('Disconnecting from Bluetooth device...');
        bluetoothDevice.gatt.disconnect();
    } else {
        notifications.info('Bluetooth device already disconnected.');
    }
    // The onDisconnected handler will clean up the rest
}

export async function send(data: string) {
    if (!rxCharacteristic) {
        console.error('RX characteristic not available.');
        return;
    }

    try {
        const encoder = new TextEncoder();
        notifications.info('Sending:' + data);
        await rxCharacteristic.writeValueWithoutResponse(encoder.encode(data)); // Use writeValue for acknowledged writes if needed
    } catch (error) {
        console.error('Failed to send data:', error);
    }
}

function handleNotifications(event: Event) {
    const value = (event.target as BluetoothRemoteGATTCharacteristic).value;
    if (value) {
        const decoder = new TextDecoder('utf-8');
        const data = decoder.decode(value);
        notifications.info('Received:' + data);
        receivedData.set(data);
    }
}

function onDisconnected() {
    notifications.info('Bluetooth device disconnected.');
    isConnected.set(false);
    bluetoothDevice = null;
    server = null;
    txCharacteristic = null;
    rxCharacteristic = null;
}
