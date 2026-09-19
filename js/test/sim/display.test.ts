// ==================== 测试：显示单位偏好与格式化 ====================
import { describe, it } from 'node:test';
import assert from 'node:assert';
import { formatSpeed, display } from '../../shared/display.js';

describe('formatSpeed', () => {
  it('仿真单位 → km/h（默认单位）：100u = 25m/s = 90km/h', () => {
    assert.strictEqual(formatSpeed(100, 'kmh'), '90.0 km/h');
    assert.strictEqual(formatSpeed(0, 'kmh'), '0.0 km/h');
  });

  it('仿真单位 → m/s', () => {
    assert.strictEqual(formatSpeed(100, 'ms'), '25.0 m/s');
    assert.strictEqual(formatSpeed(70, 'ms'), '17.5 m/s');
  });
});

describe('display 偏好', () => {
  it('默认单位为 km/h', () => {
    assert.strictEqual(display.speedUnit, 'kmh');
  });
});
