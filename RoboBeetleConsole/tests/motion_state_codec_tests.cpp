#include "protocol/MotionStateCodec.h"
#include "protocol/PacketCodec.h"
#include "robot/MotionStateCsvLogger.h"
#include <QCoreApplication>

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <iostream>

static void writeCompatibilityCsv(const QString &directory) {
    rb::MotionStateCsvLogger logger;
    assert(logger.start(directory, "task19_cpp_logger_v2"));
    robobeetle::protocol::CpgParametersSnapshot parameters;
    parameters.request_sequence = 0x1234;
    parameters.version = 7;
    logger.recordParameters(1, 901000, parameters);
    logger.recordParameters(1, 901000, parameters); // one event for the same version
    for (unsigned i = 0; i < 600; ++i) {
        rb::MotionStateRecord record;
        record.linkEpoch = 1;
        record.batchSeq = i;
        record.mcuTxMs = record.sample.mcu_ms = 1000 + i * 50;
        record.mcuUnwrappedMs = record.sample.mcu_ms;
        record.piRxMs = record.mcuTxMs + 900000;
        record.samplePiMs = record.piRxMs;
        record.clockErrorMs = 0;
        auto &sample = record.sample;
        sample.backend = 1;
        sample.active_mode = sample.target_mode = sample.state = 1;
        sample.cpg_param_version = 7;
        sample.gyro_valid = sample.angle_valid = sample.phase_valid = true;
        sample.angle_age_ms = sample.phase_age_ms = 0;
        // Synthetic development signal: 40 samples per real-phase 2 s cycle.
        sample.phase_u16 = static_cast<std::uint16_t>((i % 40) * 65536 / 40);
        sample.phase_fr_u16 = sample.phase_rr_u16 = sample.phase_rl_u16 = sample.phase_u16;
        sample.gyro_tenth_dps = {{12, -5, 33}};
        logger.record(record);
    }
    const auto path = logger.filePath();
    logger.stop();
    assert(logger.lastError().isEmpty());
    std::cout << "Actual MotionStateCsvLogger fixture: " << path.toStdString() << '\n';
}

int main(int argc, char **argv) {
    QCoreApplication app(argc, argv);
    for (unsigned count = 0; count <= 2; ++count) {
        rb::MotionStateBatch batch{};
        batch.sample_count = count;
        batch.mcu_tx_ms = 12345;
        batch.samples[0].gyro_tenth_dps[2] = -256;
        batch.samples[0].phase_valid = true;
        const auto payload = rb::encodeMotionStateBatch(batch);
        assert(payload && payload->size() == 16 + 22 * count);
        const auto decoded = rb::decodeMotionStateBatch(*payload);
        assert(decoded && decoded->sample_count == count && decoded->mcu_tx_ms == 12345);
        if (count)
            assert(decoded->samples[0].gyro_tenth_dps[2] == -256 &&
                   decoded->samples[0].phase_valid);
        const auto wire =
            rb::PacketCodec::encodeWire({rb::MessageType::MotionStateBatch, 7, *payload});
        assert(wire.size() == 28 + 22 * count);
        assert(rb::PacketCodec::decodeWire(wire.first(wire.size() - 1)).ok());
        const auto telemetry = robobeetle::gateway::encode_motion_state_telemetry(
            {1, 123456789, 0, rb::motionStateBytes(*payload)});
        assert(telemetry);
        const auto qtelemetry =
            QByteArray(reinterpret_cast<const char *>(telemetry->data()), telemetry->size());
        assert(rb::decodeMotionStateTelemetry(qtelemetry)->pi_rx_ms == 123456789);
    }
    // Fixed C/Pi/Qt contract: legacy 22B sample preserved, v2 appends 13B.
    const auto v1 = QByteArray::fromHex(
        "01013412000100007856341209000000"
        "04030201feff0300fcff0500faff00804bff07000800");
    const auto v2 = QByteArray::fromHex(
        "02013412000100007856341209000000"
        "04030201feff0300fcff0500faff00804bff07000800"
        "00016745000000111122223333");
    auto golden = rb::decodeMotionStateBatch(v1);
    assert(golden && *rb::encodeMotionStateBatch(*golden) == v1);
    golden->schema_version = 2;
    golden->samples[0].stop_reason = 1;
    golden->samples[0].cpg_param_version = 0x4567;
    golden->samples[0].phase_fr_u16 = 0x1111;
    golden->samples[0].phase_rr_u16 = 0x2222;
    golden->samples[0].phase_rl_u16 = 0x3333;
    const auto encoded = rb::encodeMotionStateBatch(*golden);
    assert(encoded && *encoded == v2 && encoded->size() == 51);
    const auto decoded = rb::decodeMotionStateBatch(v2);
    assert(decoded && decoded->schema_version == 2 && decoded->samples[0].cpg_param_version == 0x4567);
    const auto wire = rb::PacketCodec::encodeWire({rb::MessageType::MotionStateBatch, 8, v2});
    assert(rb::PacketCodec::decodeWire(wire.first(wire.size() - 1)).ok());
    const auto telemetry = robobeetle::gateway::encode_motion_state_telemetry(
        {1, 901000, 0, rb::motionStateBytes(v2)});
    assert(telemetry && telemetry->size() == 67);
    const auto wrapper = QByteArray(reinterpret_cast<const char *>(telemetry->data()), telemetry->size());
    assert(rb::decodeMotionStateTelemetry(wrapper)->batch_payload == rb::motionStateBytes(v2));
    if (app.arguments().size() == 3 && app.arguments()[1] == "--csv-output") {
        writeCompatibilityCsv(app.arguments()[2]);
    } else {
        assert(app.arguments().size() == 1);
    }
    std::cout << "Qt motion state codec: v1 wrappers, v1/v2 goldens and P2/RBRP framing passed\n";
}
