// ==================== 消息总线 ====================
//
// 【什么是消息总线（Message Bus）？】
// 消息总线是一种"发布-订阅"（Publish-Subscribe，Pub/Sub）设计模式。
// 类似于广播电台：
//   - 发布者（Publisher）向某个"频道/主题"发送消息，不需要知道谁在收听
//   - 订阅者（Subscriber）注册自己对某个主题的兴趣，收到消息后执行回调
//   - 发布者和订阅者完全解耦——通过主题名称间接通信，不需要互相引用
//
// 【为什么在自动驾驶系统中使用消息队列？】
// 真实自动驾驶系统（如 ROS/ROS2、DDS、LCM）广泛使用消息通信：
//   1. 解耦：感知/规划/控制可以是独立进程，各自独立开发、测试、部署
//   2. 可扩展：可以随时增加订阅者（如数据记录器、监控面板），不影响现有模块
//   3. 可回放：记录所有消息后可以离线回放，用于调试和回归测试
//   4. 容错：某个模块崩溃不影响其他模块
//
// 【本实现的简化】
// 进程内同步消息总线（所有回调在当前调用栈中同步执行），
// 模拟了真实系统中"消息分发"的概念，但省略了：
//   - 序列化/反序列化（内存中直接传递引用）
//   - 网络传输（单进程内直接函数调用）
//   - 异步调度（同步执行，保证确定性顺序）

type TopicHandler = (data: any) => void;

export class MessageBus {
  name: string;
  debug: boolean;
  /** 核心数据结构：Map<主题名, Set<回调函数>> */
  private _topics: Map<string, Set<TopicHandler>>;

  constructor(name = 'default', debug = false) {
    this.name = name;
    this.debug = debug;
    this._topics = new Map();
  }

  /**
   * 订阅某个主题
   *
   * 【设计约定】
   * - 同一个函数可以被多次订阅（Set 去重保证不重复调用）
   * - 返回取消订阅函数（类似 RxJS 的 Subscription.unsubscribe()）
   *
   * @param topic 主题名称（如 'tick', 'perception:lidar'）
   * @param fn    回调函数，接收消息数据作为参数
   * @returns 取消订阅函数
   */
  subscribe(topic: string, fn: TopicHandler): () => void {
    if (!this._topics.has(topic)) {
      this._topics.set(topic, new Set());
    }
    const subscribers = this._topics.get(topic)!;
    subscribers.add(fn);

    if (this.debug) {
      console.log(`[${this.name}] SUBSCRIBE  ${topic}  (共 ${subscribers.size} 个订阅者)`);
    }

    return () => {
      subscribers.delete(fn);
      if (this.debug) {
        console.log(`[${this.name}] UNSUBSCRIBE ${topic}  (剩余 ${subscribers.size} 个订阅者)`);
      }
      if (subscribers.size === 0) {
        this._topics.delete(topic);
      }
    };
  }

  /**
   * 向某个主题发布消息
   *
   * 【同步执行】
   * 所有订阅者的回调在当前调用栈中同步执行。保证了消息的
   * 确定性顺序：先订阅的回调先被调用。
   *
   * 【错误隔离】
   * 单个订阅者抛出的异常会被 catch 并打印到 console.error，
   * 不会阻止其他订阅者收到消息。一个模块崩溃不应该导致整个系统挂掉。
   */
  publish(topic: string, data: any): void {
    const subscribers = this._topics.get(topic);
    if (!subscribers || subscribers.size === 0) {
      if (this.debug) {
        console.log(`[${this.name}] PUBLISH   ${topic}  →  (无订阅者，消息丢弃)`);
      }
      return;  // 无人订阅，消息被丢弃（不是错误）
    }

    if (this.debug) {
      const payload = typeof data === 'object' ? JSON.stringify(data).slice(0, 80) : data;
      console.log(`[${this.name}] PUBLISH   ${topic}  →  ${subscribers.size} 个订阅者  |  ${payload}`);
    }

    for (const fn of subscribers) {
      try {
        fn(data);
      } catch (err) {
        console.error(`[${this.name}] 订阅者处理 "${topic}" 时出错:`, err);
      }
    }
  }

  /** 取消某个主题的所有订阅者 */
  clear(topic: string): void {
    if (this._topics.has(topic)) {
      this._topics.delete(topic);
      if (this.debug) console.log(`[${this.name}] CLEAR     ${topic}`);
    }
  }

  /** 获取某个主题的订阅者数量（调试用） */
  subscriberCount(topic: string): number {
    const subs = this._topics.get(topic);
    return subs ? subs.size : 0;
  }

  /** 列出所有有订阅者的主题（调试用） */
  listTopics(): string[] {
    return Array.from(this._topics.keys());
  }
}
