// 端子定義
const int arefPin = 0;
const int currentPin1 = 1;
const int currentPin2 = 2;
const int currentPin3 = 3;
const int currentPin4 = 4;
const int currentPin5 = 5;

const int outPin = 2;  // Debug チェック出力Pin 

// 通信設定
#define SER_SPEED  (115200)

// 商用電源周波数
#define POWER_FREQ        (60)

// １サイクルあたりのサンプル数
#define NUMBER_OF_SAMPLES (24)

// サンプリング間隔(マイクロ秒)
#define SAMPLING_PERIOD   (1000000/(POWER_FREQ * NUMBER_OF_SAMPLES))

// デバッグ用
#define DEBUG 0

// 実効電流 
//float Vrms;
float Irms1;
float Irms2;
float Irms3;
float Irms4;
float Irms5;
//float Watt;

// サンプリング用バッファ
int VASamples[NUMBER_OF_SAMPLES*10];

void calcWatt(void)
{
//#define kVT    (88.99)    // 実測にもとづく係数
#define kCT    (100.0 * 0.99 / 3000.0) // R * 係数 / 巻き数

  unsigned long t1,t2;
  int i,r,a11,a21,a31,a41,a51,a12,a22,a32,a42,a52;

  t1 = micros();

  // １サイクル分のAD値をサンプリング
  for(i=0; i<NUMBER_OF_SAMPLES; i++){

    r = analogRead(arefPin);
    a11 = analogRead(currentPin1);
    a21 = analogRead(currentPin2);
    a31 = analogRead(currentPin3);
    a41 = analogRead(currentPin4);
    a51 = analogRead(currentPin5);
    a52 = analogRead(currentPin5);
    a42 = analogRead(currentPin4);
    a32 = analogRead(currentPin3);
    a22 = analogRead(currentPin2);
    a12 = analogRead(currentPin1);

    VASamples[(i*10)+0] = a11 - r;
    VASamples[(i*10)+1] = a21 - r;
    VASamples[(i*10)+2] = a31 - r;
    VASamples[(i*10)+3] = a41 - r;
    VASamples[(i*10)+4] = a51 - r;
    VASamples[(i*10)+5] = a52 - r;
    VASamples[(i*10)+6] = a42 - r;
    VASamples[(i*10)+7] = a32 - r;
    VASamples[(i*10)+8] = a22 - r;
    VASamples[(i*10)+9] = a12 - r; 

    do {
      t2 = micros();
    } 
    while((t2 - t1) < SAMPLING_PERIOD);
    t1 += SAMPLING_PERIOD;
  }

  // １サイクル分の電圧と電流、電力を加算
//  Vrms = 0;
  Irms1 = 0;
  Irms2 = 0;
  Irms3 = 0;
  Irms4 = 0;
  Irms5 = 0;
//  Watt = 0;

  for(i=0; i<NUMBER_OF_SAMPLES; i++){
    a11 = VASamples[(i*10)+0];
    a21 = VASamples[(i*10)+1];
    a31 = VASamples[(i*10)+2];
    a41 = VASamples[(i*10)+3];
    a51 = VASamples[(i*10)+4];
    a52 = VASamples[(i*10)+5];
    a42 = VASamples[(i*10)+6];
    a32 = VASamples[(i*10)+7];
    a22 = VASamples[(i*10)+8];
    a12 = VASamples[(i*10)+9]; 
      
//    float vv = ((((v1+v2)/2) * 5.0) / 1024) * kVT;
    float aa1 = ((((a11+a12)/2) * 5.0) / 1024) / kCT;
    float aa2 = ((((a21+a22)/2) * 5.0) / 1024) / kCT;
    float aa3 = ((((a31+a32)/2) * 5.0) / 1024) / kCT;
    float aa4 = ((((a41+a42)/2) * 5.0) / 1024) / kCT;
    float aa5 = ((((a51+a52)/2) * 5.0) / 1024) / kCT;
        
//    Vrms += vv * vv;
    Irms1 += aa1 * aa1;
    Irms2 += aa2 * aa2;
    Irms3 += aa3 * aa3;
    Irms4 += aa4 * aa4;
    Irms5 += aa5 * aa5;    
//    Watt += vv * aa;
  }

  // 2乗平均平方根(rms)を求める
//  Vrms = sqrt(Vrms / NUMBER_OF_SAMPLES);
  Irms1 = sqrt(Irms1 / NUMBER_OF_SAMPLES);
  Irms2 = sqrt(Irms2 / NUMBER_OF_SAMPLES);
  Irms3 = sqrt(Irms3 / NUMBER_OF_SAMPLES);
  Irms4 = sqrt(Irms4 / NUMBER_OF_SAMPLES);
  Irms5 = sqrt(Irms5 / NUMBER_OF_SAMPLES);

  // 平均電力を求める
//  Watt = Watt / NUMBER_OF_SAMPLES;
}

//float watt_hour;
//float vrms_sum;
float irms_sum1;
float irms_sum2;
float irms_sum3;
float irms_sum4;
float irms_sum5;
//float watt_sum;
//float abs_watt;
float test_clock;
int watt_samples;
unsigned long last_update;

void setup()
{
  Serial.begin(SER_SPEED);
//  watt_hour     = 0;
//  vrms_sum      = 0;
  irms_sum1      = 0;
  irms_sum2      = 0;
  irms_sum3      = 0;
  irms_sum4      = 0;
  irms_sum5      = 0;
//  watt_sum      = 0;
  watt_samples  = 0;
  test_clock    = 0;
  last_update   = millis();

  pinMode(outPin, OUTPUT);
}

void loop()
{
  unsigned long curr_time;

  // 電力を計算
  calcWatt();

  // 1秒分加算する
//  vrms_sum += Vrms;
  irms_sum1 += Irms1;
  irms_sum2 += Irms2;
  irms_sum3 += Irms3;
  irms_sum4 += Irms4;
  irms_sum5 += Irms5;
//  watt_sum += Watt;
  watt_samples++;

  // 0.１秒経過したらシリアルに出力
  curr_time = millis();
  if( (curr_time - last_update) > 100 ){
#if DEBUG
    for(int i=0; i<NUMBER_OF_SAMPLES; i++){
      Serial.print(VASamples[(i*10)+0]);
      Serial.print('\t');
      Serial.print(VASamples[(i*10)+1]);
      Serial.print('\t');      
      Serial.print(VASamples[(i*10)+2]);
      Serial.print('\t');
      Serial.print(VASamples[(i*10)+3]);
      Serial.print('\t');
      Serial.println(VASamples[(i*10)+4]);
      Serial.print('\t');
      Serial.print(VASamples[(i*10)+5]);
      Serial.print('\t');
      Serial.print(VASamples[(i*10)+6]);
      Serial.print('\t');
      Serial.print(VASamples[(i*10)+7]);
      Serial.print('\t');
      Serial.print(VASamples[(i*10)+8]);
      Serial.print('\t');
      Serial.println(VASamples[(i*10)+9]);
    }
#endif
//    vrms_sum /= watt_samples;
    irms_sum1 /= watt_samples;
    irms_sum2 /= watt_samples;
    irms_sum3 /= watt_samples;
    irms_sum4 /= watt_samples;
    irms_sum5 /= watt_samples;
    //    watt_sum /= watt_samples;

    digitalWrite(outPin, HIGH);  //Debug用出力

    Serial.print(test_clock);
    Serial.print(" ");
    test_clock += 0.1;
    
//    Serial.print(vrms_sum);
//    Serial.print("Vrms");
//    Serial.print(",");
    
    Serial.print(irms_sum1);
//    Serial.print("Irms_1");
    Serial.print(",");
    Serial.print(irms_sum2);
//    Serial.print("Irms_2");
    Serial.print(",");
    Serial.print(irms_sum3);
//    Serial.print("Irms_3");
    Serial.print(",");
    Serial.print(irms_sum4);
//    Serial.print("Irms_4");
    Serial.print(",");
    Serial.println(irms_sum5);
//    Serial.print("Irms_5");
//    Serial.print(",");
    
//    Serial.print(vrms_sum * irms_sum);
//    Serial.print("VA, ");

//    abs_watt = abs(watt_sum);
//    Serial.print(abs_watt);
//    Serial.print(", ");
//    Serial.print(watt_sum);
//    Serial.print("W, ");

    // 力率 = 有効電力 / 皮相電力
//    Serial.print((watt_sum * 100) / (vrms_sum * irms_sum));
//    Serial.print("%, ");

    // 積算Whを求める
//    watt_hour += watt_sum / 3600.0;
//    Serial.print(watt_hour);
//    Serial.println("Wh");

    digitalWrite(outPin, LOW);

    watt_samples = 0;
    irms_sum1 = irms_sum2 = irms_sum3 = irms_sum4 = irms_sum5 = 0;

    last_update = curr_time;
  }
}
