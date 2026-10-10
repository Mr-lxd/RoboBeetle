#include "robot/MotionStateCsvLogger.h"
#include <QDir>
#include <QSignalBlocker>
#include <QStringList>
#include <numbers>

namespace rb {
namespace {
const QByteArray header = "schema_version,session_id,link_epoch,batch_seq,fragment_index,sample_index,mcu_ms,mcu_unwrapped_ms,mcu_tx_ms,pi_rx_ms,sample_pi_ms,clock_error_ms,gyro_x,gyro_y,gyro_z,roll,pitch,gait_phase_u16,gait_phase,backend,active_mode,target_mode,coordination,motion_state,in_transition,gyro_valid,angle_valid,phase_valid,angle_age_ms,phase_age_ms,sampler_drop_total,gateway_drop_total,batch_gap_total,fragment_gap_total,record_type,control_mode,stop_reason,cpg_param_version,effective_throttle,effective_turn,effective_pitch,cpg_phases_valid,phase_fr_u16,phase_rr_u16,phase_rl_u16,phase_fl_u16,phase_fr,phase_rr,phase_rl,phase_fl,cpg_front_amp_deg,cpg_rear_amp_deg,cpg_nominal_period_s,cpg_beta,cpg_front_rear_phase_deg,cpg_left_right_phase_deg,cpg_coupling_strength,cpg_coupling_mask\n";
QString num(double n) { return QString::number(n, 'g', 17); }
}
MotionStateCsvLogger::MotionStateCsvLogger(QObject *parent) : QObject(parent) {
    timer_.setInterval(250);
    connect(&timer_, &QTimer::timeout, this, [this] {
        if (file_.isOpen() && !file_.flush()) fail(file_.errorString());
    });
}
MotionStateCsvLogger::~MotionStateCsvLogger() { const QSignalBlocker b(this); stop(); }
bool MotionStateCsvLogger::start(const QString &directory, const QString &sessionId) {
    if (isRecording()) return true;
    error_.clear(); session_ = sessionId; bytes_ = 0; recordedParameters_.reset();
    if (!QDir().mkpath(directory)) { fail("Cannot create motion CSV directory"); return false; }
    file_.setFileName(QDir(directory).absoluteFilePath("motion_state_" + sessionId + ".csv"));
    if (!file_.open(QIODevice::WriteOnly | QIODevice::NewOnly)) { fail(file_.errorString()); return false; }
    if (!append(header)) return false;
    timer_.start(); return true;
}
void MotionStateCsvLogger::fail(const QString &error) {
    error_ = error; timer_.stop(); file_.close(); emit failed();
}
bool MotionStateCsvLogger::append(const QByteArray &bytes) {
    if (bytes.size() > 32 * 1024 * 1024 - bytes_) { fail("Motion CSV 32 MiB limit reached"); return false; }
    if (file_.write(bytes) != bytes.size()) { fail(file_.errorString()); return false; }
    bytes_ += bytes.size(); return true;
}
void MotionStateCsvLogger::record(const MotionStateRecord &r) {
    if (!isRecording()) return;
    const auto &s = r.sample;
    QStringList f{"motion-state-csv-v2", session_, QString::number(r.linkEpoch),
        QString::number(r.batchSeq), QString::number(r.fragmentIndex), QString::number(r.sampleIndex),
        QString::number(s.mcu_ms), QString::number(r.mcuUnwrappedMs), QString::number(r.mcuTxMs),
        QString::number(r.piRxMs), num(r.samplePiMs), num(r.clockErrorMs)};
    for (auto v : s.gyro_tenth_dps) f << num(v / 10.0);
    f << num(s.roll_centidegrees / 100.0) << num(s.pitch_centidegrees / 100.0)
      << QString::number(s.phase_u16) << num(s.phase_u16 * 2.0 * std::numbers::pi / 65536)
      << QString::number(s.backend) << QString::number(s.active_mode) << QString::number(s.target_mode)
      << QString::number(s.coordination) << QString::number(s.state) << QString::number(s.transition)
      << QString::number(s.gyro_valid) << QString::number(s.angle_valid) << QString::number(s.phase_valid)
      << QString::number(s.angle_age_ms) << QString::number(s.phase_age_ms)
      << QString::number(r.samplerDropTotal) << QString::number(r.gatewayDropTotal)
      << QString::number(r.batchGapTotal) << QString::number(r.fragmentGapTotal);
    f << "sample" << QString::number(s.control_mode) << QString::number(s.stop_reason) << QString::number(s.cpg_param_version)
      << num(s.effective_throttle/100.0) << num(s.effective_turn/100.0) << num(s.effective_pitch/100.0)
      << QString::number(s.backend==1 && s.phase_valid);
    const quint16 phases[]{s.phase_fr_u16,s.phase_rr_u16,s.phase_rl_u16,s.phase_u16};
    for (const auto v:phases) f << (s.backend==1 && s.phase_valid?QString::number(v):QString{});
    for (const auto v:phases) f << (s.backend==1 && s.phase_valid?num(v*2.0*std::numbers::pi/65536):QString{});
    for (int i=0;i<8;++i) f << QString{};
    append((f.join(',') + '\n').toUtf8());
}
void MotionStateCsvLogger::recordParameters(quint32 epoch, quint64 piRxMs, const robobeetle::protocol::CpgParametersSnapshot &s) {
    if (!isRecording() || recordedParameters_==std::make_pair(epoch,s.version)) return;
    recordedParameters_=std::make_pair(epoch,s.version);
    QStringList f; for (int i=0;i<58;++i) f << QString{};
    f[0]="motion-state-csv-v2"; f[1]=session_; f[2]=QString::number(epoch); f[9]=QString::number(piRxMs);
    f[34]="cpg_parameters"; f[37]=QString::number(s.version);
    const auto &p=s.parameters;
    const double values[]{p.front_amp,p.rear_amp,p.nominal_period,p.beta,p.front_rear_phase,p.left_right_phase,p.coupling_strength};
    for (int i=0;i<7;++i) f[50+i]=num(values[i]); f[57]=QString::number(p.coupling_mask);
    append((f.join(',')+'\n').toUtf8());
}
void MotionStateCsvLogger::stop() {
    timer_.stop();
    if (file_.isOpen() && !file_.flush()) { fail(file_.errorString()); return; }
    file_.close();
}
void MotionStateCsvLogger::recordTotals(const MotionStateMonitor &m) {
    if (!isRecording() || !m.linkEpoch()) return;
    QStringList f;
    for (int i = 0; i < 58; ++i) f << QString{};
    f[0] = "motion-state-csv-v2"; f[1] = session_; f[2] = QString::number(*m.linkEpoch());
    f[34]="totals";
    f[30] = QString::number(m.samplerDropTotal()); f[31] = QString::number(m.gatewayDropTotal());
    f[32] = QString::number(m.batchGapTotal()); f[33] = QString::number(m.fragmentGapTotal());
    append((f.join(',') + '\n').toUtf8());
}
} // namespace rb
