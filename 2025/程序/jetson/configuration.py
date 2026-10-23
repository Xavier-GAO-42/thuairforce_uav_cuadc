import json
import os
import logging
from typing import Any, Optional

logger = logging.getLogger(__name__)

class Config:
    """
    配置文件管理类，用于读取和访问JSON格式的配置
    
    特性:
    - 初始化时自动加载配置文件
    - 支持点符号访问嵌套配置 (如: config.get('database.host'))
    - 类型安全访问方法 (get_string, get_int, get_bool等)
    - 支持默认值
    - 自动缓存配置数据
    
    示例:
        config = Config('config.json')
        db_host = config.get_string('database.host', 'localhost')
        debug_mode = config.get_bool('debug_mode', False)
    """
    
    def __init__(self, config_path: str):
        """
        初始化配置对象并加载配置文件
        
        参数:
            config_path: 配置文件的路径
            
        异常:
            FileNotFoundError: 配置文件不存在
            json.JSONDecodeError: 配置文件格式错误
        """
        self._config_path = config_path
        self._config_data = {}
        self._load_config()
    
    def _load_config(self) -> None:
        """加载并解析配置文件"""
        if not os.path.exists(self._config_path):
            raise FileNotFoundError(f"配置文件不存在: {self._config_path}")
        
        if not os.path.isfile(self._config_path):
            raise ValueError(f"路径不是文件: {self._config_path}")
        
        try:
            with open(self._config_path, 'r', encoding='utf-8') as f:
                self._config_data = json.load(f)
        except json.JSONDecodeError as e:
            raise json.JSONDecodeError(
                f"配置文件解析错误 ({self._config_path}): {e.msg}",
                e.doc,
                e.pos
            ) from e
        except PermissionError:
            raise PermissionError(f"无权限读取文件: {self._config_path}")
    
    def reload(self) -> None:
        """重新加载配置文件"""
        self._load_config()
    
    def _get_nested_value(self, key_path: str) -> Any:
        """
        使用点符号访问嵌套配置值
        
        参数:
            key_path: 点分隔的键路径 (如 'database.credentials.username')
            
        返回:
            配置值
            
        异常:
            KeyError: 如果路径不存在
        """
        keys = key_path.split('.')
        current = self._config_data
        
        for key in keys:
            if not isinstance(current, dict) or key not in current:
                logger.warning(f"[configuration warning] 配置键 '{key_path}' 不存在")
                raise KeyError(f"配置键 '{key_path}' 不存在")
            current = current[key]
        
        return current
    
    def get(self, key_path: str, default: Any = None) -> Any:
        """
        获取配置值，支持点符号访问嵌套键
        
        参数:
            key_path: 点分隔的键路径
            default: 键不存在时返回的默认值
            
        返回:
            配置值或默认值
        """
        try:
            return self._get_nested_value(key_path)
        except KeyError:
            return default
    
    def get_string(self, key_path: str, default: str = '') -> str:
        """获取字符串类型配置值"""
        value = self.get(key_path, default)
        return str(value) if value is not None else default
    
    def get_int(self, key_path: str, default: int = 0) -> int:
        """获取整数类型配置值"""
        try:
            value = self.get(key_path)
            return int(value) if value is not None else default
        except (TypeError, ValueError):
            return default
    
    def get_float(self, key_path: str, default: float = 0.0) -> float:
        """获取浮点数类型配置值"""
        try:
            value = self.get(key_path)
            return float(value) if value is not None else default
        except (TypeError, ValueError):
            return default
    
    def get_bool(self, key_path: str, default: bool = False) -> bool:
        """获取布尔类型配置值"""
        value = self.get(key_path)
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.lower() in ('true', 'yes', '1', 'on')
        if isinstance(value, (int, float)):
            return bool(value)
        return default
    
    def get_list(self, key_path: str, default: Optional[list] = None) -> list:
        """获取列表类型配置值"""
        value = self.get(key_path)
        if value is None:
            return default or []
        return list(value) if isinstance(value, (list, tuple)) else [value]
    
    def get_dict(self, key_path: str, default: Optional[dict] = None) -> dict:
        """获取字典类型配置值"""
        value = self.get(key_path)
        if value is None:
            return default or {}
        return dict(value) if isinstance(value, dict) else default or {}
    
    def get_raw_data(self) -> dict:
        """获取原始配置数据字典"""
        return self._config_data
    
    def __contains__(self, key_path: str) -> bool:
        """检查配置键是否存在"""
        try:
            self._get_nested_value(key_path)
            return True
        except KeyError:
            return False
    
    @property
    def config_path(self) -> str:
        """获取配置文件路径"""
        return self._config_path