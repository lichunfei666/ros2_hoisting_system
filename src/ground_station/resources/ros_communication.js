// 简化版ROS通信模块
class ROS {
    constructor(options) {
        this.url = options.url;
        this.socket = null;
        this.subscribers = {};
        this.onConnectionCallbacks = [];
        this.onCloseCallbacks = [];
        this.onErrorCallbacks = [];
        this.isConnected = false;
    }

    connect() {
        this.socket = new WebSocket(this.url);
        
        this.socket.onopen = () => {
            console.log('ROS连接已建立');
            this.isConnected = true;
            this.onConnectionCallbacks.forEach(callback => callback());
        };

        this.socket.onmessage = (event) => {
            const data = JSON.parse(event.data);
            
            if (data.op === 'publish' && this.subscribers[data.topic]) {
                this.subscribers[data.topic].forEach(callback => callback(data.msg));
            }
        };

        this.socket.onclose = () => {
            console.log('ROS连接已关闭');
            this.isConnected = false;
            this.onCloseCallbacks.forEach(callback => callback());
        };

        this.socket.onerror = (error) => {
            console.error('ROS连接错误:', error);
            this.onErrorCallbacks.forEach(callback => callback(error));
        };
    }

    on(event, callback) {
        if (event === 'connection') {
            this.onConnectionCallbacks.push(callback);
        } else if (event === 'close') {
            this.onCloseCallbacks.push(callback);
        } else if (event === 'error') {
            this.onErrorCallbacks.push(callback);
        }
    }

    disconnect() {
        if (this.socket) {
            this.socket.close();
        }
    }
}

class Topic {
    constructor(options) {
        this.ros = options.ros;
        this.name = options.name;
        this.messageType = options.messageType;
        this.id = Math.random().toString(36).substr(2, 9);
    }

    subscribe(callback) {
        if (!this.ros.subscribers[this.name]) {
            this.ros.subscribers[this.name] = [];
        }
        this.ros.subscribers[this.name].push(callback);

        const subscribeMsg = {
            op: 'subscribe',
            id: this.id,
            topic: this.name,
            type: this.messageType
        };

        if (this.ros.socket && this.ros.socket.readyState === WebSocket.OPEN) {
            this.ros.socket.send(JSON.stringify(subscribeMsg));
        }
    }

    unsubscribe() {
        const unsubscribeMsg = {
            op: 'unsubscribe',
            id: this.id,
            topic: this.name
        };

        if (this.ros.socket && this.ros.socket.readyState === WebSocket.OPEN) {
            this.ros.socket.send(JSON.stringify(unsubscribeMsg));
        }

        delete this.ros.subscribers[this.name];
    }

    publish(message) {
        const publishMsg = {
            op: 'publish',
            topic: this.name,
            msg: message
        };

        if (this.ros.socket && this.ros.socket.readyState === WebSocket.OPEN) {
            this.ros.socket.send(JSON.stringify(publishMsg));
        } else {
            console.error('WebSocket未连接，无法发布消息');
        }
    }
}

class Message {
    constructor(data) {
        Object.assign(this, data);
    }
}

class Service {
    constructor(options) {
        this.ros = options.ros;
        this.name = options.name;
        this.serviceType = options.serviceType;
    }

    callService(request, callback) {
        const id = Math.random().toString(36).substr(2, 9);
        const serviceMsg = {
            op: 'call_service',
            id: id,
            service: this.name,
            args: request
        };

        const handleResponse = (event) => {
            const data = JSON.parse(event.data);
            if (data.op === 'service_response' && data.id === id) {
                callback(data.values, data.result);
                this.ros.socket.removeEventListener('message', handleResponse);
            }
        };

        this.ros.socket.addEventListener('message', handleResponse);

        if (this.ros.socket && this.ros.socket.readyState === WebSocket.OPEN) {
            this.ros.socket.send(JSON.stringify(serviceMsg));
        } else {
            console.error('WebSocket未连接，无法调用服务');
        }
    }
}

class ServiceRequest {
    constructor(data) {
        Object.assign(this, data);
    }
}

class Time {
    constructor(secs = 0, nsecs = 0) {
        this.secs = secs;
        this.nsecs = nsecs;
    }

    static now() {
        const now = new Date();
        const secs = Math.floor(now.getTime() / 1000);
        const nsecs = now.getMilliseconds() * 1000000;
        return new Time(secs, nsecs);
    }

    // Add more methods to match the expected API
    toString() {
        return `${this.secs}.${this.nsecs.toString().padStart(9, '0')}`;
    }

    toSec() {
        return this.secs + this.nsecs / 1000000000;
    }

    toNsec() {
        return this.secs * 1000000000 + this.nsecs;
    }
}

// 为了兼容原代码，创建ROSLIB对象
window.ROSLIB = {
    Ros: ROS,
    Topic: Topic,
    Message: Message,
    Service: Service,
    ServiceRequest: ServiceRequest,
    Time: Time
};

// 为了兼容直接使用ros.Time的代码
window.ros = window.ros || {};
window.ros.Time = Time;