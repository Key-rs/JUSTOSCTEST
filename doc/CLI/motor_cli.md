# 概述

在 **JUSTOS** 中，兼容原有的 **JUSTFW** 电机驱动框架，并新增了 **MOTOR Manage CLI** 工具，用于支持电机数据的存储和配置。

通过该工具，用户可以动态设置电机参数、将参数保存到 Flash 中，并实现对电机的运动控制。

---

# 原理解释

在保持原有静态电机参数设置的基础上，新增了从 Flash 中读取电机参数并同步到电机实例的功能。这样可以确保在以下情况下电机仍能正常工作：
1. **Flash 数据异常**：即使 Flash 中的数据损坏或丢失，电机仍可使用默认静态参数运行。
2. **不使用 Flash**：在某些场景下，用户可以选择不依赖 Flash 存储，直接使用默认参数。

通过这种方式，系统既保留了灵活性，又保证了可靠性。

---

# MOTOR Manage CLI

电机管理 CLI 工具（Motor Manage CLI）提供了便捷的命令行接口，用于动态配置和控制电机。

## 特性

- **动态设置电机参数**：支持通过命令行修改电机的运行参数。
- **参数持久化**：将配置的参数保存到 Flash 中，支持掉电保存。
- **电机运动控制**：提供简单的命令来启动、停止和调整电机的运动状态。

---

## 命令设计

为了简化使用并适配 FreeRTOS CLI 模块，命令设计遵循以下原则：
1. **简洁明了**：命令尽量短小，易于记忆。
2. **功能明确**：每个命令专注于单一功能，避免复杂组合。
3. **错误友好**：命令输入错误时提供清晰的提示信息。

### 命令列表

#### 1. 设置电机参数
动态设置电机的运行参数：
```bash
motor set <param> <value>
```
- **参数说明**：
  - `<param>`：参数名称（如 `speed`、`direction` 等）。
  - `<value>`：参数值，**不能包含空格**。
- **示例**：
  ```bash
  motor set speed 1000
  motor set direction forward
  ```

#### 2. 获取电机参数
查询并显示指定电机参数的当前值：
```bash
motor get <param>
```
- **参数说明**：
  - `<param>`：参数名称。
- **示例**：
  ```bash
  motor get speed
  ```

#### 3. 保存电机参数
将当前设置的电机参数保存到 Flash 中：
```bash
motor save
```
- **示例**：
  ```bash
  motor save
  ```

#### 4. 加载电机参数
从 Flash 中加载电机参数并同步到电机实例中：
```bash
motor load
```
- **示例**：
  ```bash
  motor load
  ```

#### 5. 控制电机运动
启动或停止电机：
```bash
motor control <action>
```
- **参数说明**：
  - `<action>`：动作类型，支持 `start` 和 `stop`。
- **示例**：
  ```bash
  motor control start
  motor control stop
  ```

#### 6. 列出所有参数
列出当前电机的所有参数及其值：
```bash
motor list
```
- **示例输出**：
  ```
  speed=1000
  direction=forward
  acceleration=500
  ```

#### 7. 重置电机参数
恢复电机参数到默认值：
```bash
motor reset
```
- **注意**：此操作不可逆，请谨慎使用。
- **示例**：
  ```bash
  motor reset
  ```

---

## 示例场景

### 场景 1：初始化电机参数
在设备首次启动时，可以通过 CLI 初始化电机参数：
```bash
motor set speed 1000
motor set direction forward
motor set acceleration 500
motor save
```

### 场景 2：动态调整电机速度
在运行过程中，动态调整电机的速度：
```bash
motor set speed 1500
motor control start
```

### 场景 3：查看当前电机配置
在调试或维护时，查看当前的所有电机参数：
```bash
motor list
```

### 场景 4：恢复默认配置
在需要恢复出厂设置时，重置电机参数：
```bash
motor reset
motor save
```

---

## 注意事项

1. **存储容量限制**  
   Flash 的存储容量有限，请合理规划电机参数的数量和大小，避免超出存储限制。

2. **掉电保护**  
   确保在修改电机参数后及时执行 `motor save` 操作，以防止因意外断电导致数据丢失。

3. **Value 格式限制**  
   - 参数值中**不能包含空格**，否则会导致命令解析失败。
   - 不需要使用双引号包裹参数值。
   - 示例：
     ```bash
     # 正确
     motor set description ThisIsATestValue

     # 错误（包含空格）
     motor set description This is a test value
     ```

4. **错误处理**  
   在使用 CLI 工具时，若出现错误提示，请检查输入参数是否正确，或确认 Flash 设备是否正常工作。

---

## 贡献与反馈

如果您在使用过程中发现任何问题，或希望提出改进建议，请随时提交 Issue 或 Pull Request。我们欢迎您的贡献！

---

通过以上设计，MOTOR Manage CLI 工具既保持了简洁性，又提供了足够的功能性，能够满足用户对电机参数配置和控制的需求，同时适配 FreeRTOS CLI 模块的要求。