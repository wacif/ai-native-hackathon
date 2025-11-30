---
sidebar_position: 3
---

# باب 2: ایکچویٹرز اور موومنٹ کنٹرول

ماحول کو محسوس کرنے کے بعد، روبوٹس کو عمل کرنے کے قابل ہونا چاہیے۔ یہ باب ایکچویٹرز کی تلاش کرتا ہے جو جسمانی حرکت کو ممکن بناتے ہیں اور کنٹرول سسٹمز جو انہیں ہم آہنگ کرتے ہیں۔

## جائزہ

ایکچویٹرز روبوٹ کے "پٹھے" ہیں، جو برقی، ہائیڈرالک، یا نیومیٹک توانائی کو مکینیکل حرکت میں تبدیل کرتے ہیں۔ ہیومنائڈ روبوٹس کے لیے، ایکچویٹرز کو فراہم کرنا لازمی ہے:

- **زیادہ ٹارک سے وزن کا تناسب**: متحرک حرکت کے لیے مضبوط لیکن ہلکا وزن
- **درست کنٹرول**: درست پوزیشن اور رفتار ٹریکنگ
- **کمپلائنس**: انسانوں اور ماحول کے ساتھ محفوظ تعامل
- **کارکردگی**: محدود بیٹری پاور پر طویل آپریشن ٹائم

## ایکچویٹرز کی اقسام

### الیکٹرک موٹرز

الیکٹرک موٹرز ہیومنائڈ روبوٹکس میں سب سے عام ایکچویٹرز ہیں ان کی درستگی اور کنٹرول میں آسانی کی وجہ سے۔

**DC برشڈ موٹرز**
- سادہ، سستی، اور کنٹرول میں آسان
- برش وقت کے ساتھ گھس جاتے ہیں
- کم لاگت ایپلیکیشنز کے لیے اچھی

**برش لیس DC (BLDC) موٹرز**
- برشڈ موٹرز سے زیادہ موثر اور طویل عرصے تک چلنے والی
- الیکٹرانک اسپیڈ کنٹرولرز (ESCs) کی ضرورت
- ڈرونز اور جدید روبوٹس میں عام

**سروو موٹرز**
- پوزیشن کی درستگی کے لیے انٹیگریٹڈ فیڈ بیک کنٹرول
- مختلف سائز اور ٹارک ریٹنگز میں دستیاب
- معیاری ہوبی سروز: محدود رینج (±90°)
- صنعتی سروز: زیادہ درستگی، متعدد گھماؤ

**مثال: سروو کنٹرول**
```python
import time
from adafruit_servokit import ServoKit

# سروو کنٹرولر انیشیلائز کریں (PCA9685)
kit = ServoKit(channels=16)

def move_servo_smooth(servo_channel, target_angle, duration=1.0, steps=50):
    """
    سروو کو موجودہ سے ہدف کے زاویے تک ہموار طریقے سے منتقل کریں
    
    Args:
        servo_channel: سروو چینل نمبر (0-15)
        target_angle: ہدف کا زاویہ ڈگریوں میں (0-180)
        duration: حرکت مکمل کرنے کا وقت (سیکنڈز)
        steps: انٹرپولیشن اسٹیپس کی تعداد
    """
    current_angle = kit.servo[servo_channel].angle
    if current_angle is None:
        current_angle = 90  # ڈیفالٹ شروعاتی پوزیشن
    
    # ہموار ٹریجیکٹری جنریٹ کریں
    angles = [current_angle + (target_angle - current_angle) * i / steps 
              for i in range(steps + 1)]
    
    delay = duration / steps
    
    for angle in angles:
        kit.servo[servo_channel].angle = angle
        time.sleep(delay)

# استعمال کی مثال
move_servo_smooth(servo_channel=0, target_angle=120, duration=2.0)
```

### ہائیڈرالک ایکچویٹرز

ہائیڈرالک سسٹمز حرکت اور قوت پیدا کرنے کے لیے دبے ہوئے مائع کا استعمال کرتے ہیں۔

**فوائد:**
- بہت زیادہ طاقت سے وزن کا تناسب
- بہت زیادہ قوت پیدا کر سکتے ہیں
- قدرتی طور پر کمپلائنٹ اور بیک ڈرائیویبل

**نقصانات:**
- پمپس، ریزروائرز، اور پیچیدہ پلمبنگ کی ضرورت
- لیکیج کا امکان
- شور اور دیکھ بھال کی ضروریات

**ایپلیکیشنز:**
- Boston Dynamics کا Atlas روبوٹ
- بڑے صنعتی مینیپولیٹرز
- ہیوی ڈیوٹی تعمیراتی روبوٹس

### نیومیٹک ایکچویٹرز

نیومیٹک سسٹمز ایکچویشن کے لیے کمپریسڈ ہوا کا استعمال کرتے ہیں۔

**فوائد:**
- فطری طور پر محفوظ (کمپریسیبل میڈیم)
- کمپلائنٹ اور نرم تعامل
- سادہ اور ہلکے وزن

**نقصانات:**
- ہائیڈرالکس سے کم فورس ڈینسٹی
- درست طریقے سے کنٹرول مشکل
- ایئر کمپریسر کی ضرورت

**ایپلیکیشنز:**
- سافٹ روبوٹکس
- گرپرز اور اینڈ ایفیکٹرز
- انسان محفوظ کولیبوریٹو روبوٹس

### سیریز الاسٹک ایکچویٹرز (SEAs)

SEAs میں موٹر اور لوڈ کے درمیان ایک کمپلائنٹ اسپرنگ شامل ہے، جو فراہم کرتا ہے:

- **فورس کنٹرول**: فورس کا تخمینہ لگانے کے لیے اسپرنگ ڈیفلیکشن ماپیں
- **شاک ابزورپشن**: اسپرنگ گیئرز کو اثرات سے بچاتی ہے
- **انرجی اسٹوریج**: الاسٹک عنصر توانائی ذخیرہ اور جاری کر سکتا ہے
- **محفوظ تعامل**: کمپلائنس رابطے کے دوران چوٹ کو روکتی ہے

**مثال حساب:**
```python
def sea_force_estimation(motor_position, load_position, spring_stiffness):
    """
    سیریز الاسٹک ایکچویٹر میں فورس کا تخمینہ
    
    Args:
        motor_position: موٹر شافٹ پوزیشن (ریڈینز)
        load_position: لوڈ/جوائنٹ پوزیشن (ریڈینز)
        spring_stiffness: اسپرنگ کانسٹنٹ (Nm/rad)
    
    Returns:
        تخمینی ٹارک (Nm)
    """
    spring_deflection = motor_position - load_position
    torque = spring_stiffness * spring_deflection
    return torque

# مثال
motor_pos = 1.5  # ریڈینز
joint_pos = 1.3  # ریڈینز
k_spring = 100  # Nm/rad

force = sea_force_estimation(motor_pos, joint_pos, k_spring)
print(f"تخمینی ٹارک: {force:.2f} Nm")
```

## موٹر کنٹرول تکنیکیں

### پوزیشن کنٹرول

پوزیشن کنٹرول ایکچویٹر کو مطلوبہ زاویے یا پوزیشن پر لے جاتا ہے۔

**PID کنٹرول** (Proportional-Integral-Derivative):

```python
class PIDController:
    def __init__(self, kp, ki, kd, output_limits=(-1, 1)):
        self.kp = kp  # متناسب گین
        self.ki = ki  # انٹیگرل گین
        self.kd = kd  # ڈیریویٹو گین
        self.output_limits = output_limits
        
        self.prev_error = 0
        self.integral = 0
    
    def update(self, setpoint, measured_value, dt):
        """
        کنٹرول آؤٹ پٹ کا حساب لگائیں
        
        Args:
            setpoint: مطلوبہ قدر
            measured_value: موجودہ ماپی گئی قدر
            dt: ٹائم اسٹیپ (سیکنڈز)
        
        Returns:
            کنٹرول سگنل
        """
        # ایرر کا حساب لگائیں
        error = setpoint - measured_value
        
        # متناسب ٹرم
        p_term = self.kp * error
        
        # انٹیگرل ٹرم (اینٹی ونڈ اپ کے ساتھ)
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # ڈیریویٹو ٹرم
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative
        
        # کل آؤٹ پٹ
        output = p_term + i_term + d_term
        
        # آؤٹ پٹ کو کلیمپ کریں
        output = max(min(output, self.output_limits[1]), 
                    self.output_limits[0])
        
        # اگلی تکرار کے لیے محفوظ کریں
        self.prev_error = error
        
        return output

# جوائنٹ کنٹرول کے لیے استعمال کی مثال
pid = PIDController(kp=5.0, ki=0.1, kd=0.5)

target_angle = 90  # ڈگری
current_angle = 45  # ڈگری
dt = 0.01  # 100 Hz کنٹرول لوپ

control_signal = pid.update(target_angle, current_angle, dt)
```

### ویلوسٹی کنٹرول

ویلوسٹی کنٹرول مطلوبہ زاویائی یا لکیری رفتار برقرار رکھتا ہے۔

**ایپلیکیشنز:**
- ہموار ٹریجیکٹریز
- ہم آہنگ ملٹی جوائنٹ حرکات
- جھٹکے دار حرکت سے بچنا

### ٹارک/فورس کنٹرول

ٹارک کنٹرول ایکچویٹر کی فورس آؤٹ پٹ کو براہ راست کمانڈ کرتا ہے۔

**فوائد:**
- ماحول کے ساتھ کمپلائنٹ تعامل
- درست فورس ایپلیکیشن
- توانائی کی بچت والی حرکت

**امپیڈینس کنٹرول:**
```python
import numpy as np

def impedance_controller(position, velocity, target_position, 
                        target_velocity, stiffness, damping):
    """
    کمپلائنٹ تعامل کے لیے امپیڈینس کنٹرول
    
    ورچوئل اسپرنگ-ڈیمپر سسٹم کی طرح برتاؤ کرتا ہے
    
    Args:
        position: موجودہ پوزیشن
        velocity: موجودہ ویلوسٹی
        target_position: مطلوبہ پوزیشن
        target_velocity: مطلوبہ ویلوسٹی
        stiffness: ورچوئل اسپرنگ اسٹفنس
        damping: ورچوئل ڈیمپنگ کوایفیشینٹ
    
    Returns:
        مطلوبہ ٹارک/فورس
    """
    position_error = target_position - position
    velocity_error = target_velocity - velocity
    
    # ورچوئل اسپرنگ-ڈیمپر سسٹم
    force = stiffness * position_error + damping * velocity_error
    
    return force

# مثال: کمپلائنٹ ریچنگ
current_pos = np.array([0.5, 0.3, 0.8])
current_vel = np.array([0.0, 0.0, 0.0])
target_pos = np.array([0.7, 0.4, 0.9])
target_vel = np.array([0.0, 0.0, 0.0])

K = 50  # اسٹفنس
B = 10  # ڈیمپنگ

force_command = impedance_controller(current_pos, current_vel, 
                                    target_pos, target_vel, K, B)
print(f"فورس کمانڈ: {force_command}")
```

## ٹریجیکٹری جنریشن

ہموار ٹریجیکٹریز جھٹکے دار حرکت کو روکتی ہیں اور مکینیکل اجزاء پر ٹوٹ پھوٹ کم کرتی ہیں۔

### پوائنٹ ٹو پوائنٹ ٹریجیکٹریز

**ٹریپیزوائڈل ویلوسٹی پروفائل:**
- مستقل ایکسیلیریشن فیز
- مستقل ویلوسٹی کروز فیز
- مستقل ڈی سیلیریشن فیز

**S-Curve (جرک لمیٹڈ) پروفائل:**
- ایکسیلیریشن کی تبدیلی کی شرح کو محدود کرتا ہے
- زیادہ ہموار حرکت، کم وائبریشن

```python
import numpy as np
import matplotlib.pyplot as plt

def trapezoidal_trajectory(start, end, v_max, a_max, dt=0.01):
    """
    ٹریپیزوائڈل ویلوسٹی ٹریجیکٹری جنریٹ کریں
    
    Args:
        start: شروعاتی پوزیشن
        end: اختتامی پوزیشن
        v_max: زیادہ سے زیادہ ویلوسٹی
        a_max: زیادہ سے زیادہ ایکسیلیریشن
        dt: ٹائم اسٹیپ
    
    Returns:
        time, position, velocity, acceleration arrays
    """
    distance = abs(end - start)
    direction = np.sign(end - start)
    
    # زیادہ سے زیادہ ویلوسٹی تک پہنچنے کا وقت
    t_accel = v_max / a_max
    
    # ایکسیلیریشن اور ڈی سیلیریشن کے دوران فاصلہ
    d_accel = 0.5 * a_max * t_accel**2
    
    # چیک کریں کہ ہم زیادہ سے زیادہ ویلوسٹی تک پہنچتے ہیں
    if 2 * d_accel > distance:
        # مثلثی پروفائل (کبھی v_max تک نہیں پہنچتے)
        t_accel = np.sqrt(distance / a_max)
        v_max = a_max * t_accel
        t_cruise = 0
    else:
        # ٹریپیزوائڈل پروفائل
        t_cruise = (distance - 2 * d_accel) / v_max
    
    t_total = 2 * t_accel + t_cruise
    
    # ٹریجیکٹری جنریٹ کریں
    time = np.arange(0, t_total, dt)
    position = np.zeros_like(time)
    velocity = np.zeros_like(time)
    acceleration = np.zeros_like(time)
    
    for i, t in enumerate(time):
        if t < t_accel:
            # ایکسیلیریشن فیز
            position[i] = start + direction * 0.5 * a_max * t**2
            velocity[i] = direction * a_max * t
            acceleration[i] = direction * a_max
        elif t < t_accel + t_cruise:
            # کروز فیز
            t_cruise_elapsed = t - t_accel
            position[i] = start + direction * (d_accel + v_max * t_cruise_elapsed)
            velocity[i] = direction * v_max
            acceleration[i] = 0
        else:
            # ڈی سیلیریشن فیز
            t_decel = t - t_accel - t_cruise
            position[i] = end - direction * 0.5 * a_max * (t_accel - t_decel)**2
            velocity[i] = direction * a_max * (t_accel - t_decel)
            acceleration[i] = -direction * a_max
    
    return time, position, velocity, acceleration

# استعمال کی مثال
time, pos, vel, acc = trapezoidal_trajectory(
    start=0, end=100, v_max=50, a_max=100
)
```

### ملٹی جوائنٹ کوآرڈینیشن

ہیومنائڈ روبوٹس کے لیے، متعدد جوائنٹس کو ہم آہنگی سے حرکت کرنی چاہیے۔

**جوائنٹ اسپیس پلاننگ:**
- ہر جوائنٹ کی آزادانہ پلاننگ کریں
- سادہ لیکن غیر موثر کارٹیشین پاتھس کا سبب بن سکتا ہے

**کارٹیشین اسپیس پلاننگ:**
- 3D اسپیس میں اینڈ ایفیکٹر پاتھ کی پلاننگ کریں
- جوائنٹ اینگلز کمپیوٹ کرنے کے لیے انورس کائنیمیٹکس استعمال کریں
- زیادہ بدیہی لیکن کمپیوٹیشنل طور پر مہنگا

## پاور ٹرانسمیشن

موٹرز سے جوائنٹس تک پاور پہنچانے کے لیے مکینیکل ٹرانسمیشن سسٹمز کی ضرورت ہے۔

### گیئرز

**سپر گیئرز:**
- سادہ، موثر، متوازی شافٹس
- شور مچا سکتے ہیں

**پلینیٹری گیئرز:**
- کمپیکٹ پیکج میں زیادہ ریڈکشن ریشوز
- عام طور پر روبوٹ جوائنٹس میں استعمال

**ہارمونک ڈرائیوز (سٹرین ویو گیئرز):**
- بہت زیادہ ریڈکشن ریشوز (50:1 سے 200:1)
- صفر بیک لیش
- کمپیکٹ اور ہلکے وزن
- مہنگے لیکن پریسیژن روبوٹس کے لیے صنعتی معیار

### بیلٹس اور پولیز

- گیئرز سے ہلکے
- لمبے فاصلوں پر پاور ٹرانسمٹ کر سکتے ہیں
- ہلکے روبوٹک آرمز میں استعمال

### ڈائریکٹ ڈرائیو

- موٹر براہ راست جوائنٹ سے جڑی (کوئی گیئر باکس نہیں)
- صفر بیک لیش، انتہائی بیک ڈرائیویبل
- ہائی ٹارک موٹرز کی ضرورت
- جدید تحقیقی روبوٹس میں استعمال

## ہیٹ مینجمنٹ

موٹرز آپریشن کے دوران گرمی پیدا کرتی ہیں، جس کا انتظام کرنا ضروری ہے:

**کولنگ حکمت عملی:**
- پیسو: ہیٹ سنکس، تھرمل کنڈکشن
- ایکٹو: فینز، لیکوڈ کولنگ سسٹمز
- تھرمل ڈیزائن: ہیٹ پائپس، فیز چینج میٹیریلز

**تھرمل ماڈلنگ:**
```python
def motor_temperature_simple(ambient_temp, power_loss, thermal_resistance, 
                             thermal_capacitance, dt, current_temp):
    """
    موٹر درجہ حرارت کے لیے سادہ فرسٹ آرڈر تھرمل ماڈل
    
    Args:
        ambient_temp: ماحول کا درجہ حرارت (°C)
        power_loss: موٹر پاور لاس/ہیٹ جنریشن (W)
        thermal_resistance: ماحول میں تھرمل مزاحمت (°C/W)
        thermal_capacitance: تھرمل کپیسیٹینس (J/°C)
        dt: ٹائم اسٹیپ (سیکنڈز)
        current_temp: موجودہ موٹر درجہ حرارت (°C)
    
    Returns:
        نیا موٹر درجہ حرارت (°C)
    """
    # ماحول میں ہیٹ فلو
    heat_flow = (current_temp - ambient_temp) / thermal_resistance
    
    # خالص ہیٹ تبدیلی
    heat_net = power_loss - heat_flow
    
    # درجہ حرارت کی تبدیلی
    temp_change = (heat_net / thermal_capacitance) * dt
    
    new_temp = current_temp + temp_change
    
    return new_temp
```

## کیس اسٹڈی: Boston Dynamics Atlas

Atlas سب سے جدید ہیومنائڈ روبوٹس میں سے ایک ہے، جو جدید ترین ایکچویشن کا مظاہرہ کرتا ہے:

- **ہائیڈرالک ایکچویشن**: 28 ہائیڈرالک طور پر چلنے والے جوائنٹس
- **کسٹم والوز**: ریسپانسیو کنٹرول کے لیے ہائی بینڈوڈتھ سروو والوز
- **آن بورڈ پاور**: روبوٹ پر رکھی بیٹری اور ہائیڈرالک پمپ
- **فورس کنٹرول**: متحرک توازن کے لیے ہول باڈی ٹارک کنٹرول
- **کارکردگی**: دوڑ سکتا ہے، چھلانگ لگا سکتا ہے، بیک فلپ کر سکتا ہے، اور کھردرے علاقے میں نیویگیٹ کر سکتا ہے

## خلاصہ

ایکچویٹرز اور کنٹرول سسٹمز روبوٹ ذہانت اور جسمانی عمل کے درمیان پل ہیں:

- الیکٹرک موٹرز سب سے عام ہیں، لاگت، درستگی، اور طاقت میں مختلف تجارتی موازنے کے ساتھ
- کنٹرول حکمت عملیاں سادہ پوزیشن کنٹرول سے لے کر نفیس ٹارک کنٹرول تک ہیں
- کارکردگی اور طویل عمر کے لیے ہموار ٹریجیکٹری جنریشن ضروری ہے
- پاور ٹرانسمیشن سسٹمز زیادہ قوت ایپلیکیشنز کے لیے موٹر ٹارک کو بڑھاتے ہیں
- مسلسل آپریشن کے لیے تھرمل مینجمنٹ اہم ہے

اگلے باب میں، ہم دریافت کریں گے کہ یہ ایکچویٹرز اور سینسرز روبوٹک ذہانت کے لیے AI الگورتھمز میں کیسے اکٹھے ہوتے ہیں۔

---

**پچھلا**: [روبوٹ سینسرز اور ادراک ←](./chapter1.md) | **اگلا**: [روبوٹکس کے لیے AI الگورتھمز →](./chapter3.md)

## مشقیں

1. PID کنٹرولر نافذ کریں اور پوزیشن کنٹرول کے لیے گینز ٹیون کریں
2. P, PI، اور PID کنٹرولرز کے اسٹیپ ریسپانس کا موازنہ کریں
3. ہیومنائڈ روبوٹ آرم جوائنٹ کے لیے گیئر ریڈکشن سسٹم ڈیزائن کریں
4. ٹریپیزوائڈل اور S-curve ٹریجیکٹریز نافذ اور ویژولائز کریں
5. ڈیٹا شیٹ سے موٹر کی تھرمل ٹائم کانسٹنٹ کا حساب لگائیں
