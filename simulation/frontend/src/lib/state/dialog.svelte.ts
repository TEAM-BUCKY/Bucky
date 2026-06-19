import type { Component } from "svelte";

interface Popup {
    component: Component;
    data: Record<string, any>;
}

export class DialogState {
    id = $state<number>();

    component = $state<Component>();
    data = $state<Record<string, any>>();

    onclose: () => void = () => {};

    constructor(
        id: number,
        props: Popup,
        public close: (value?: any) => void,
    ) {
        this.id = id;

        this.component = props.component;
        this.data = props.data;
    }
}

class DialogsState {
    popups = $state<DialogState[]>([]);
    count = 0;

    getID() {
        this.count++;
        if (this.count > 1000) this.count = 0;

        return this.count;
    }

    open(props: Popup) {
        return new Promise((resolve) => {
            if (this.popups.find((popup) => popup.component === props.component)) {
                resolve(undefined);
                return;
            }

            const id = this.getID();
            this.popups.push(
                new DialogState(id, props, (value: any) => {
                    this.popups = this.popups.filter((popup) => popup.id !== id);
                    resolve(value);
                }),
            );
        });
    }

    async setup() {

    }

    clear() {
        this.popups = [];
    }
}

export default new DialogsState();