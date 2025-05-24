import { writable } from 'svelte/store';
import { notifications } from '../components/toasts/notifications';

const bluetoothEvents = ['open', 'close', 'error', 'message', 'unresponsive'] as const;
type BluetoothEvent = (typeof bluetoothEvents)[number];

export const SERVICE_UUID = '6e400001-b5a3-f393-e0a9-e50e24dcca9e';
const CHARACTERISTIC_TX_UUID = '6e400003-b5a3-f393-e0a9-e50e24dcca9e';
const CHARACTERISTIC_RX_UUID = '6e400002-b5a3-f393-e0a9-e50e24dcca9e';

function createBluetoothService() {
    let listeners = new Map<string, Set<(data?: unknown) => void>>();
    const { subscribe, set } = writable(false);
    const reconnectTimeoutTime = 5000;
    let unresponsiveTimeoutId: number;
    let reconnectTimeoutId: number;

    let bluetoothDevice: BluetoothDevice | null = null;
    let server: BluetoothRemoteGATTServer | null = null;
    let txCharacteristic: BluetoothRemoteGATTCharacteristic | null = null;
    let rxCharacteristic: BluetoothRemoteGATTCharacteristic | null = null;

    function disconnect(reason: BluetoothEvent, event?: Event) {
        if (bluetoothDevice?.gatt?.connected) {
            bluetoothDevice.gatt.disconnect();
        }
        set(false);
        clearTimeout(unresponsiveTimeoutId);
        clearTimeout(reconnectTimeoutId);
        listeners.get(reason)?.forEach(listener => listener(event));

        if (reason !== 'close') {
            reconnectTimeoutId = setTimeout(connect, reconnectTimeoutTime);
        }

        bluetoothDevice = null;
        server = null;
        txCharacteristic = null;
        rxCharacteristic = null;
    }

    async function connect() {
        if (!navigator.bluetooth) {
            notifications.error('Bluetooth not supported');
            return;
        }

        try {
            notifications.info('Requesting Bluetooth device...');
            bluetoothDevice = await navigator.bluetooth.requestDevice({
                acceptAllDevices: true,
                optionalServices: [SERVICE_UUID]
            });

            if (!bluetoothDevice || !bluetoothDevice.gatt) {
                throw new Error('No device selected or GATT not available.');
            }

            server = await bluetoothDevice.gatt.connect();
            const service = await server.getPrimaryService(SERVICE_UUID);
            txCharacteristic = await service.getCharacteristic(CHARACTERISTIC_TX_UUID);
            rxCharacteristic = await service.getCharacteristic(CHARACTERISTIC_RX_UUID);

            await txCharacteristic.startNotifications();
            txCharacteristic.addEventListener('characteristicvaluechanged', handleNotifications);
            bluetoothDevice.addEventListener('gattserverdisconnected', () => disconnect('close'));

            set(true);
            clearTimeout(reconnectTimeoutId);
            listeners.get('open')?.forEach(listener => listener());
            resetUnresponsiveCheck();

            for (const event of listeners.keys()) {
                if (bluetoothEvents.includes(event as BluetoothEvent)) continue;
                subscribeToEvent(event);
            }
        } catch (error) {
            notifications.error('Bluetooth connection failed: ' + error);
            disconnect('error', error as Event);
        }
    }

    function handleNotifications(event: Event) {
        resetUnresponsiveCheck();
        const value = (event.target as BluetoothRemoteGATTCharacteristic).value;
        if (!value) return;

        const decoder = new TextDecoder('utf-8');
        let data = decoder.decode(value);

        // Parse the data following the socket format: type/event[payload]
        if (data[0] === '2') {
            // Event message type
            data = data.substring(1);
            if (!data) return;

            let event = data.substring(data.indexOf('/') + 1, data.indexOf('['));
            let payload = data.substring(data.indexOf('[') + 1, data.lastIndexOf(']'));

            try {
                payload = JSON.parse(payload);
            } catch (error) {}

            if (event) {
                listeners.get(event)?.forEach(listener => listener(payload));
            }
        }
    }

    function resetUnresponsiveCheck() {
        clearTimeout(unresponsiveTimeoutId);
        unresponsiveTimeoutId = setTimeout(() => disconnect('unresponsive'), reconnectTimeoutTime);
    }

    function sendEvent(event: string, data: unknown) {
        if (!rxCharacteristic) return;
        try {
            const message = `2/${event}[${JSON.stringify(data)}]`;
            const encoder = new TextEncoder();
            rxCharacteristic.writeValueWithoutResponse(encoder.encode(message));
        } catch (error) {
            console.error('Failed to send data:', error);
            listeners.get('error')?.forEach(listener => listener(error));
        }
    }

    function subscribeToEvent(event: string) {
        if (!rxCharacteristic) return;
        const message = `0/${event}`;
        const encoder = new TextEncoder();
        rxCharacteristic.writeValueWithoutResponse(encoder.encode(message));
    }

    function unsubscribeToEvent(event: string) {
        if (!rxCharacteristic) return;
        const message = `1/${event}`;
        const encoder = new TextEncoder();
        rxCharacteristic.writeValueWithoutResponse(encoder.encode(message));
    }

    function unsubscribe(event: string, listener?: (data: any) => void) {
        let eventListeners = listeners.get(event);
        if (!eventListeners) return;

        if (!eventListeners.size) {
            unsubscribeToEvent(event);
        }
        if (listener) {
            eventListeners.delete(listener);
        } else {
            listeners.delete(event);
        }
    }

    return {
        subscribe,
        sendEvent,
        init: connect,
        on: <T>(event: string, listener: (data: T) => void): (() => void) => {
            let eventListeners = listeners.get(event);
            if (!eventListeners) {
                if (!bluetoothEvents.includes(event as BluetoothEvent)) {
                    subscribeToEvent(event);
                }
                eventListeners = new Set();
                listeners.set(event, eventListeners);
            }
            eventListeners.add(listener as (data: any) => void);

            return () => {
                unsubscribe(event, listener);
            };
        },
        off: (event: string, listener?: (data: any) => void) => {
            unsubscribe(event, listener);
        }
    };
}

export const bluetooth = createBluetoothService();
