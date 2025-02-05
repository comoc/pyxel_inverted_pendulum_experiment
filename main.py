import pyxel
import math

class InvertedPendulum:
    def __init__(self):
        # ウィンドウとシミュレーションの初期設定
        pyxel.init(160, 120, title="Inverted Pendulum")
        
        # 物理パラメータ
        self.g = 9.81  # 重力加速度
        self.m = 1.0   # 振子の質量
        self.l = 30.0  # 振子の長さ（ピクセル）
        self.dt = 1/60 # 時間刻み

        # PID制御パラメータ（調整済み）
        self.Kp = 1000.0  # 比例ゲイン
        self.Ki = 0.1     # 積分ゲイン
        self.Kd = 300.0   # 微分ゲイン
        self.integral_error = 0.0
        self.prev_error = 0.0
        
        # 制御モード（True: 自動制御、False: 手動制御）
        self.auto_control = True
        
        # 振子の状態（0は右向き、π/2が上向き、πが左向き、-π/2が下向き）
        self.theta = math.pi/2 + 0.1  # 初期角度（ラジアン）- 上向きから少しずれた位置
        self.omega = 0.0  # 角速度
        self.cart_x = 80  # 台車のx座標
        self.cart_v = 0   # 台車の速度
        
        # 外乱の大きさ
        self.force = 100.0  # 制御力を増加
        
        pyxel.run(self.update, self.draw)
    
    def normalize_angle(self, angle):
        # 角度を-πからπの範囲に正規化
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def calculate_control(self):
        # 目標値（真上の状態 = -π/2）からの誤差を計算
        error = self.normalize_angle(self.theta - (-math.pi/2))
        
        # 積分項の計算（積分の制限付き）
        self.integral_error = max(-5.0, min(5.0, self.integral_error + error * self.dt))
        
        # 微分項の計算（角速度を使用）
        derivative = self.omega
        
        # PID制御の計算
        control = (self.Kp * error + 
                  self.Ki * self.integral_error + 
                  self.Kd * derivative)
        
        # 制御入力を制限
        return max(-self.force, min(self.force, control))
    
    def update(self):
        # ESCで終了
        if pyxel.btnp(pyxel.KEY_Q):
            pyxel.quit()
        
        # スペースキーで制御モード切り替え
        if pyxel.btnp(pyxel.KEY_SPACE):
            self.auto_control = not self.auto_control
            self.integral_error = 0  # 積分項をリセット
        
        # 力の計算
        force = 0
        if self.auto_control:
            # 自動制御モード
            force = self.calculate_control()
        else:
            # 手動制御モード
            if pyxel.btn(pyxel.KEY_LEFT):
                force = -self.force
            if pyxel.btn(pyxel.KEY_RIGHT):
                force = self.force
        
        # 運動方程式の計算
        # より詳細な倒立振子のモデル
        sin_theta = math.sin(self.theta)
        cos_theta = math.cos(self.theta)
        
        # 振子の運動方程式（符号を反転）
        alpha = (-self.g * cos_theta + force * sin_theta / self.m) / self.l
        self.omega += alpha * self.dt
        
        # 減衰項の追加（エネルギー損失をシミュレート）
        damping = 0.1
        self.omega *= (1.0 - damping * self.dt)
        
        self.theta += self.omega * self.dt
        
        # 台車の移動
        self.cart_v = force * self.dt / self.m
        self.cart_x += self.cart_v
        
        # 台車の位置を画面内に制限
        self.cart_x = max(20, min(140, self.cart_x))
    
    def draw(self):
        # 画面クリア
        pyxel.cls(7)
        
        # 地面の線
        pyxel.line(0, 100, 160, 100, 0)
        
        # 台車の描画
        pyxel.rect(self.cart_x - 10, 90, 20, 10, 5)
        
        # 振子の描画
        pendulum_x = self.cart_x + self.l * math.cos(self.theta)  # cosを使用
        pendulum_y = 95 - self.l * math.sin(self.theta)  # sinを使用
        pyxel.line(self.cart_x, 95, pendulum_x, pendulum_y, 0)
        pyxel.circ(pendulum_x, pendulum_y, 3, 8)
        
        # 操作説明の表示
        pyxel.text(5, 5, "LEFT/RIGHT: Apply Force", 0)
        pyxel.text(5, 15, "SPACE: Toggle Control Mode", 0)
        pyxel.text(5, 25, "Q: Quit", 0)
        
        # 制御モードの表示
        mode_text = "Auto Control" if self.auto_control else "Manual Control"
        pyxel.text(5, 35, mode_text, 0)

if __name__ == "__main__":
    InvertedPendulum()
