import { writable } from 'svelte/store';

type StateType = 'info' | 'success' | 'warning' | 'error';

type State = {
    id: string;
    type: StateType;
    message: string;
};

function createNotificationStore() {
    const state: State[] = [];
    const notifications = writable(state);
    const { subscribe } = notifications;

    function send(message: string, type: StateType = 'info', timeout: number) {
        const id = generateId();
        setTimeout(() => {
            notifications.update(state => {
                return state.filter(n => n.id !== id);
            });
        }, timeout);
        notifications.update(state => {
            return [...state, { id, type, message }];
        });
    }

    return {
        subscribe,
        send,
        error: (msg: string, timeout: number = 4000) => send(msg, 'error', timeout),
        warning: (msg: string, timeout: number = 4000) => send(msg, 'warning', timeout),
        info: (msg: string, timeout: number = 4000) => send(msg, 'info', timeout),
        success: (msg: string, timeout: number = 4000) => send(msg, 'success', timeout)
    };
}

function generateId() {
    return '_' + Math.random().toString(36).substr(2, 9);
}

export const notifications = createNotificationStore();
