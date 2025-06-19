import sys
import importlib
import subprocess
import shutil
from pathlib import Path
from typing import Optional, Tuple, Dict
import tkinter as tk
from tkinter import ttk, messagebox
from google.protobuf import descriptor_pool, message_factory
from google.protobuf.descriptor import FieldDescriptor
import serial
import serial.tools.list_ports
import time

# 全局变量存储BOARD_CONFIG常量值
BOARD_CONFIG = None

class ProtoCompiler:
    @classmethod
    def compile_and_load(cls, proto_file: str, temp_dir: Optional[str] = None, keep_generated: bool = False) -> object:
        script_dir = Path(__file__).parent.resolve()
        proto_path = (script_dir / proto_file).resolve()
        temp_dir = Path(temp_dir) if temp_dir else script_dir / "temp_proto"
        temp_dir.mkdir(exist_ok=True)

        try:
            compile_result = cls._compile_proto(proto_path, temp_dir)
            module = cls._load_module(proto_path, temp_dir)

            if not keep_generated:
                cls._cleanup(temp_dir)

            return module
        except subprocess.CalledProcessError as e:
            error_msg = cls._format_compile_error(e)
            cls._cleanup(temp_dir)
            raise RuntimeError(error_msg) from e

    @staticmethod
    def _compile_proto(proto_path: Path, temp_dir: Path):
        return subprocess.run(
            [
                "protoc",
                f"--python_out={temp_dir}",
                f"--proto_path={proto_path.parent}",
                str(proto_path)
            ],
            check=True,
            capture_output=True,
            text=True
        )

    @staticmethod
    def _load_module(proto_path: Path, temp_dir: Path):
        module_name = proto_path.stem + "_pb2"
        sys.path.insert(0, str(temp_dir))
        try:
            return importlib.import_module(module_name)
        except ImportError as e:
            raise RuntimeError(f"无法加载生成的模块: {str(e)}") from e

    @staticmethod
    def _format_compile_error(e: subprocess.CalledProcessError) -> str:
        return (
            f"Proto编译失败:\n"
            f"命令: {e.cmd}\n"
            f"错误码: {e.returncode}\n"
            f"输出: {e.stderr}"
        )

    @staticmethod
    def _cleanup(temp_dir: Path):
        try:
            shutil.rmtree(temp_dir, ignore_errors=True)
        except Exception as e: 
            print(f"清理警告: {str(e)}")


class ProtoUIBuilder:
    @staticmethod
    def create_field_widget(parent: ttk.Frame, field: FieldDescriptor, message: object) -> Tuple[ttk.Label, tk.Widget, tk.Variable]:
        label_text = field.name.replace('_', ' ').title()
        var = ProtoUIBuilder._create_variable(field, message)

        label = ttk.Label(parent, text=label_text)
        widget = ProtoUIBuilder._create_input_widget(parent, field, var)

        return label, widget, var

    @staticmethod
    def _create_variable(field: FieldDescriptor, message: object) -> tk.Variable:
        current_value = getattr(message, field.name)

        if field.type == FieldDescriptor.TYPE_BOOL:
            return tk.StringVar(value=str(current_value))
        elif field.type == FieldDescriptor.TYPE_ENUM:
            enum_cls = field.enum_type
            return tk.StringVar(value=enum_cls.values_by_number[current_value].name)
        elif field.type in (FieldDescriptor.TYPE_DOUBLE, FieldDescriptor.TYPE_FLOAT):
            return tk.DoubleVar(value=current_value)
        elif field.type in (FieldDescriptor.TYPE_INT32, FieldDescriptor.TYPE_INT64):
            return tk.IntVar(value=current_value)
        else:
            return tk.StringVar(value=str(current_value))

    @staticmethod
    def _create_input_widget(parent: ttk.Frame, field: FieldDescriptor, var: tk.Variable):
        if field.type == FieldDescriptor.TYPE_BOOL:
            return ProtoUIBuilder._create_bool_widget(parent, var)
        elif field.type == FieldDescriptor.TYPE_ENUM:
            return ProtoUIBuilder._create_enum_widget(parent, field, var)
        elif field.type in (FieldDescriptor.TYPE_DOUBLE, FieldDescriptor.TYPE_FLOAT):
            return ProtoUIBuilder._create_float_widget(parent, var)
        elif field.type in (
            FieldDescriptor.TYPE_INT32,
            FieldDescriptor.TYPE_INT64,
            FieldDescriptor.TYPE_UINT32,
            FieldDescriptor.TYPE_UINT64,
            FieldDescriptor.TYPE_FIXED32,
            FieldDescriptor.TYPE_FIXED64,
            FieldDescriptor.TYPE_SFIXED32,
            FieldDescriptor.TYPE_SFIXED64,
            FieldDescriptor.TYPE_SINT32,
            FieldDescriptor.TYPE_SINT64
        ):
            return ProtoUIBuilder._create_int_widget(parent, var)
        else:
            return ttk.Entry(parent, textvariable=var)

    @staticmethod
    def _create_bool_widget(parent, var):
        return ttk.Combobox(
            parent,
            textvariable=var,
            values=["True", "False"],
            state="readonly"
        )

    @staticmethod
    def _create_enum_widget(parent, field, var):
        enum_cls = field.enum_type
        names = [v.name for v in enum_cls.values]
        return ttk.Combobox(
            parent,
            textvariable=var,
            values=names,
            state="readonly"
        )

    @staticmethod
    def _create_float_widget(parent, var):
        widget = ttk.Spinbox(
            parent,
            from_=-999999.0,
            to=999999.0,
            increment=0.1,
            format="%.3f",
            textvariable=var
        )
        widget.configure(
            validate="key",
            validatecommand=(parent.register(ProtoUIBuilder._validate_float), '%P')
        )
        return widget

    @staticmethod
    def _create_int_widget(parent, var):
        widget = ttk.Spinbox(
            parent,
            from_=-999999,
            to=999999,
            textvariable=var
        )
        widget.configure(
            validate="key",
            validatecommand=(parent.register(ProtoUIBuilder._validate_int), '%P')
        )
        return widget

    @staticmethod
    def _validate_float(new_value: str) -> bool:
        try:
            if new_value.strip() in ("", "-", ''):
                return True
            float(new_value)
            return True
        except ValueError:
            return False

    @staticmethod
    def _validate_int(new_value: str) -> bool:
        try:
            if new_value.strip() in ("", "-", ''):
                return True
            int(new_value)
            return True
        except ValueError:
            return False


class SerialCommunicator:
    def __init__(self):
        self.ser = None

    def connect(self, port, baudrate):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            return True
        except serial.SerialException as e:
            print(f'连接串口失败: {e}')
            return False

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send_data(self, data):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(data)
                return True
            except serial.SerialException as e:
                print(f'发送数据失败: {e}')
                return False
        return False

    def receive_data(self):
        if self.ser and self.ser.is_open:
            try:
                # 设置一个缓冲区来存储所有接收到的数据
                buffer = bytearray()
                # 设置超时时间（秒）
                timeout = time.time() + 1.0  # 1秒超时
                
                while True:
                    # 读取可用的数据
                    if self.ser.in_waiting:
                        chunk = self.ser.read(self.ser.in_waiting)
                        buffer.extend(chunk)
                        # 重置超时时间
                        timeout = time.time() + 0.1  # 接收到数据后再等待0.1秒
                    
                    # 检查是否超时
                    if time.time() > timeout:
                        break
                    
                    # 短暂休眠以避免过度占用CPU
                    time.sleep(0.01)
                
                return bytes(buffer)
            except serial.SerialException as e:
                print(f'接收数据失败: {e}')
        return b''


class ProtoConfigurator(tk.Tk):
    def __init__(self, proto_module: object):
        super().__init__()
        self.proto_module = proto_module
        self.message = self._create_message_instance()
        self.field_vars: Dict[str, tk.Variable] = {}
        self.serial_comm = SerialCommunicator()
        self.port_var = tk.StringVar()
        self.baudrate_var = tk.StringVar(value='921600')

        self._setup_window()
        self._build_ui()
        self._setup_bindings()

    def _setup_window(self):
        self.title("Protobuf配置工具")
        self.geometry("650x900")
        self._configure_styles()

    def _configure_styles(self):
        style = ttk.Style()
        style.configure("TLabel", font=("Arial", 10))
        style.configure("TButton", font=("Arial", 10, "bold"))
        style.configure("Error.TLabel", foreground="red")

    def _create_message_instance(self) -> object:
        candidates = [
            cls_name for cls_name in dir(self.proto_module)
            if cls_name.endswith("Config") and cls_name[0].isupper()
        ]

        if not candidates:
            raise ValueError("未找到有效的配置消息类")

        try:
            message_cls = getattr(self.proto_module, candidates[0])
            return message_cls()
        except AttributeError as e:
            raise ValueError(f"无法实例化消息类: {str(e)}") from e

    def _build_ui(self):
        main_frame = ttk.Frame(self)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        canvas, scrollbar, scroll_frame = self._setup_scrollable_area(main_frame)

        # 串口设置UI
        serial_frame = ttk.LabelFrame(scroll_frame, text='串口设置')
        serial_frame.pack(fill=tk.X, pady=10)

        port_label = ttk.Label(serial_frame, text='串口号:')
        port_label.pack(side=tk.LEFT, padx=5)
        port_combobox = ttk.Combobox(serial_frame, textvariable=self.port_var, values=self._get_available_ports())
        port_combobox.pack(side=tk.LEFT, padx=5)

        baudrate_label = ttk.Label(serial_frame, text='波特率:')
        baudrate_label.pack(side=tk.LEFT, padx=5)
        baudrate_entry = ttk.Entry(serial_frame, textvariable=self.baudrate_var)
        baudrate_entry.pack(side=tk.LEFT, padx=5)

        connect_btn = ttk.Button(serial_frame, text='连接', command=self._connect_serial)
        connect_btn.pack(side=tk.LEFT, padx=5)

        disconnect_btn = ttk.Button(serial_frame, text='断开', command=self._disconnect_serial)
        disconnect_btn.pack(side=tk.LEFT, padx=5)

        for field in self.message.DESCRIPTOR.fields:
            label, widget, var = ProtoUIBuilder.create_field_widget(scroll_frame, field, self.message)
            label.pack(anchor=tk.W, pady=(10, 0))
            widget.pack(fill=tk.X, pady=(0, 10))
            self.field_vars[field.name] = var

        self._create_save_button(scroll_frame)

    def _get_available_ports(self):
        return [port.device for port in serial.tools.list_ports.comports()]

    def _connect_serial(self):
        port = self.port_var.get()
        baudrate = int(self.baudrate_var.get())
        if self.serial_comm.connect(port, baudrate):
            messagebox.showinfo('连接成功', '串口连接成功')
        else:
            messagebox.showerror('连接失败', '串口连接失败')

    def _disconnect_serial(self):
        self.serial_comm.disconnect()
        messagebox.showinfo('断开连接', '串口已断开')

    def _setup_scrollable_area(self, main_frame):
        canvas = tk.Canvas(main_frame)
        scrollbar = ttk.Scrollbar(main_frame, orient=tk.VERTICAL, command=canvas.yview)
        scroll_frame = ttk.Frame(canvas)

        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        canvas.create_window((0, 0), window=scroll_frame, anchor="nw")
        scroll_frame.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))

        return canvas, scrollbar, scroll_frame

    def _create_save_button(self, scroll_frame):
        button_frame = ttk.Frame(scroll_frame)
        button_frame.pack(fill=tk.X, pady=20)

        read_btn = ttk.Button(
            button_frame,
            text="读取配置",
            command=self._on_read
        )
        read_btn.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        save_btn = ttk.Button(
            button_frame,
            text="保存配置",
            command=self._on_save
        )
        save_btn.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

    def _setup_bindings(self):
        for field in self.message.DESCRIPTOR.fields:
            var = self.field_vars[field.name]
            var.trace_add("write", lambda *args, f=field, v=var: self._update_field(f, v))

    def _update_field(self, field: FieldDescriptor, var: tk.Variable):
        try:
            raw_value = var.get()
            if raw_value == '' or raw_value == "":
                return
            value = self._convert_value(field, raw_value)
            setattr(self.message, field.name, value)
        except Exception as e:
            self._handle_update_error(field, var, str(e))

    def _convert_value(self, field: FieldDescriptor, value):
        if field.type == FieldDescriptor.TYPE_BOOL:
            return str(value).lower() in ('true', '1', 't')
        elif field.type == FieldDescriptor.TYPE_ENUM:
            return self._convert_enum(field, value)
        elif field.type in (FieldDescriptor.TYPE_DOUBLE, FieldDescriptor.TYPE_FLOAT):
            return self._convert_float(value)
        elif field.type in (
            FieldDescriptor.TYPE_INT32,
            FieldDescriptor.TYPE_INT64,
            FieldDescriptor.TYPE_UINT32,
            FieldDescriptor.TYPE_UINT64,
            FieldDescriptor.TYPE_FIXED32,
            FieldDescriptor.TYPE_FIXED64,
            FieldDescriptor.TYPE_SFIXED32,
            FieldDescriptor.TYPE_SFIXED64,
            FieldDescriptor.TYPE_SINT32,
            FieldDescriptor.TYPE_SINT64
        ):
            return self._convert_int(value)
        else:
            return str(value)

    def _convert_int(self, value) -> int:
        try:
            if isinstance(value, str):
                value = value.strip()
                if value.isdigit() or (value.startswith('-') and value[1:].isdigit()):
                    return int(value)
                elif '.' in value:
                    return int(float(value))
            elif isinstance(value, (int, float)):
                return int(value)
        except (ValueError, TypeError):
            raise ValueError(f'{value} 不是有效的整数')

    def _convert_float(self, value) -> float:
        try:
            return float(value)
        except (ValueError, TypeError):
            raise ValueError(f"'{value}' 不是有效的浮点数")

    def _convert_enum(self, field, value):
        enum_cls = field.enum_type
        try:
            if isinstance(value, (int, type(enum_cls))):
                return enum_cls.values_by_number[value].number
            else:
                return getattr(enum_cls, value).number
        except AttributeError:
            try:
                return next(v for v in enum_cls.values if v.name == value).number
            except StopIteration:
                raise ValueError(f'{value} 不是有效的枚举值，可选值: {[v.name for v in enum_cls.values]} ')

    def _handle_update_error(self, field, var, error_msg):
        error_info = f'发生错误: {error_msg}'
        current_value = getattr(self.message, field.name)
        messagebox.showerror(
            "更新错误",
            f"字段 {field.name} 更新失败: {error_info}\n" \
            f"当前值将被重置为: {current_value}"
        )
        print(f"更新错误: 字段 {field.name} 更新失败: {error_info}\n当前值将被重置为: {current_value}")

        if isinstance(var, tk.StringVar):
            var.set(str(current_value))
        elif isinstance(var, (tk.IntVar, tk.DoubleVar)):
            var.set(current_value)

    def _on_save(self):
        try:
            # 删除原有的读取逻辑
            # 直接使用全局变量 BOARD_CONFIG
            if BOARD_CONFIG is None:
                raise ValueError('未成功读取 BOARD_CONFIG 常量值')

            for field in self.message.DESCRIPTOR.fields:
                self._update_field(field, self.field_vars[field.name])
                raw_value = self.field_vars[field.name].get()
                if not raw_value and raw_value != 0:
                    raise ValueError(f'字段 {field.name} 的值不能为空')

            serialized_data = self.message.SerializeToString()
            payload_length = len(serialized_data)
            # 计算 CRC16 校验码，需要实现 crc16 函数
            crc16 = self._calculate_crc16(serialized_data, BOARD_CONFIG, payload_length)

            # 构建完整的数据包
            packet = bytearray()
            packet.extend([BOARD_CONFIG & 0xff, (BOARD_CONFIG >> 8) & 0xff])
            packet.extend([payload_length & 0xff, (payload_length >> 8) & 0xff])
            packet.extend(serialized_data)
            packet.extend([crc16 & 0xff, (crc16 >> 8) & 0xff])

            if not self.serial_comm.ser or not self.serial_comm.ser.is_open:
                hex_data = ' '.join([f'{byte:02x}' for byte in packet])
                print(f'串口未连接，序列化数据（十六进制）: {hex_data}，数据长度: {len(packet)}')
            if self.serial_comm.send_data(packet):
                print('配置数据已发送')
                messagebox.showinfo('保存成功', '配置已更新到内存对象并发送到串口')
            else:
                messagebox.showerror('发送失败', '配置数据发送到串口失败')

        except Exception as e:
            messagebox.showerror('保存错误', f'配置保存失败: {str(e)}')
            print(f'保存错误: 配置保存失败: {str(e)}')
    
    def _on_read(self):
        try:
            if BOARD_CONFIG is None:
                raise ValueError('未成功读取 BOARD_CONFIG 常量值')

            # 构造读取命令
            header_bytes = bytearray([BOARD_CONFIG & 0xff, (BOARD_CONFIG >> 8) & 0xff])
            fixed_bytes = bytearray([0xFF, 0xFF])
            data_to_crc = header_bytes + fixed_bytes
            crc16 = self._calculate_crc16(data_to_crc, 0, 0)
            read_command = header_bytes + fixed_bytes + bytearray([crc16 & 0xff, (crc16 >> 8) & 0xff])

            if not self.serial_comm.ser or not self.serial_comm.ser.is_open:
                hex_data = ' '.join([f'{byte:02x}' for byte in read_command])
                print(f'串口未连接，读取命令（十六进制）: {hex_data}，数据长度: {len(read_command)}')
                return

            if self.serial_comm.send_data(read_command):
                print('读取命令已发送')
                # 接收数据
                received_data = self.serial_comm.receive_data()
                print(received_data)
                if len(received_data) < 6:
                    raise ValueError(f'接收到的数据长度（{len(received_data)}）不足，无法解析')

                # 解析数据
                received_header = (received_data[1] << 8) | received_data[0]
                if received_header != BOARD_CONFIG:
                    raise ValueError(f'接收到的header ({received_header}) 不匹配 ({BOARD_CONFIG})')

                payload_length = (received_data[3] << 8) | received_data[2]
                if len(received_data) != 6 + payload_length:
                    raise ValueError('接收到的数据长度与payload长度不匹配')

                received_crc16 = (received_data[-1] << 8) | received_data[-2]
                calculated_crc16 = self._calculate_crc16(received_data[4:-2], received_header, payload_length)
                if received_crc16 != calculated_crc16:
                    raise ValueError('CRC16校验失败')

                proto_data = received_data[4:-2]
                self.message.ParseFromString(proto_data)

                # 更新UI
                for field in self.message.DESCRIPTOR.fields:
                    value = getattr(self.message, field.name)
                    if field.type == FieldDescriptor.TYPE_ENUM:
                        enum_cls = field.enum_type
                        value = enum_cls.values_by_number[value].name
                    self.field_vars[field.name].set(value)

                messagebox.showinfo('读取成功', '配置已成功读取并更新到UI')
            else:
                messagebox.showerror('发送失败', '读取命令发送到串口失败')

        except Exception as e:
            messagebox.showerror('读取错误', f'配置读取失败: {str(e)}')
            print(f'读取错误: 配置读取失败: {str(e)}')

    def _calculate_crc16(self, data, header, payload_length):
        # 这里需要实现 CRC16 校验码的计算逻辑
        # 示例实现，需要替换为实际的计算方法
        crc = 0xFFFF
        # 计算 header 和 payload length 的 CRC
        temp_data = bytearray([header & 0xff, (header >> 8) & 0xff, payload_length & 0xff, (payload_length >> 8) & 0xff])
        temp_data.extend(data)
        for byte in temp_data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc


if __name__ == "__main__":
    # 读取BOARD_CONFIG常量值
    try:
        script_dir = Path(__file__).parent.resolve()
        proto_header_path = (script_dir / "../DataType/Proto/Base/proto_header.proto").resolve()
        with open(proto_header_path, 'r', encoding='utf-8') as f:
            proto_content = f.read()
        import re
        match = re.search(r'BOARD_CONFIG\s*=\s*(\d+);', proto_content)
        if match:
            BOARD_CONFIG = int(match.group(1))
        else:
            raise ValueError('未找到 BOARD_CONFIG 常量值')
    except Exception as e:
        print(f'读取 BOARD_CONFIG 常量值失败: {e}')
    try:
        proto_module = ProtoCompiler.compile_and_load(
            "../DataType/Proto/Config/Board/board_config.proto",
            keep_generated=False
        )

        app = ProtoConfigurator(proto_module)
        app.mainloop()
    except Exception as e:
        messagebox.showerror(
            "启动失败",
            f"应用程序初始化失败:\n{str(e)}"
        )
        sys.exit(1)