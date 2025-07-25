from typing import Callable, TypeAlias, Union

import numpy as np
from aspn23.image import Image, ImageCameraModel, ImageImageType
from aspn23.measurement_accumulated_distance_traveled import (
    MeasurementAccumulatedDistanceTraveled,
    MeasurementAccumulatedDistanceTraveledErrorModel,
)
from aspn23.measurement_altitude import (
    MeasurementAltitude,
    MeasurementAltitudeErrorModel,
    MeasurementAltitudeReference,
)
from aspn23.measurement_angular_velocity import (
    MeasurementAngularVelocity,
    MeasurementAngularVelocityErrorModel,
    MeasurementAngularVelocityImuType,
    MeasurementAngularVelocityReferenceFrame,
)
from aspn23.measurement_angular_velocity_1d import (
    MeasurementAngularVelocity1D,
    MeasurementAngularVelocity1DErrorModel,
    MeasurementAngularVelocity1DSensorType,
)
from aspn23.measurement_attitude_2d import (
    MeasurementAttitude2D,
    MeasurementAttitude2DErrorModel,
    MeasurementAttitude2DReferenceFrame,
)
from aspn23.measurement_attitude_3d import (
    MeasurementAttitude3D,
    MeasurementAttitude3DErrorModel,
    MeasurementAttitude3DReferenceFrame,
)
from aspn23.measurement_barometer import (
    MeasurementBarometer,
    MeasurementBarometerErrorModel,
)
from aspn23.measurement_delta_position import (
    MeasurementDeltaPosition,
    MeasurementDeltaPositionErrorModel,
    MeasurementDeltaPositionReferenceFrame,
)
from aspn23.measurement_delta_range import (
    MeasurementDeltaRange,
    MeasurementDeltaRangeErrorModel,
)
from aspn23.measurement_delta_range_to_point import (
    MeasurementDeltaRangeToPoint,
    MeasurementDeltaRangeToPointErrorModel,
)
from aspn23.measurement_direction_2d_to_points import MeasurementDirection2DToPoints
from aspn23.measurement_direction_3d_to_points import MeasurementDirection3DToPoints
from aspn23.measurement_direction_of_motion_2d import (
    MeasurementDirectionOfMotion2D,
    MeasurementDirectionOfMotion2DErrorModel,
    MeasurementDirectionOfMotion2DReference,
)
from aspn23.measurement_direction_of_motion_3d import (
    MeasurementDirectionOfMotion3D,
    MeasurementDirectionOfMotion3DErrorModel,
    MeasurementDirectionOfMotion3DReferenceFrame,
)
from aspn23.measurement_frequency_difference import (
    MeasurementFrequencyDifference,
    MeasurementFrequencyDifferenceErrorModel,
)
from aspn23.measurement_heading import (
    MeasurementHeading,
    MeasurementHeadingErrorModel,
    MeasurementHeadingReference,
)
from aspn23.measurement_IMU import MeasurementImu, MeasurementImuImuType
from aspn23.measurement_magnetic_field import (
    MeasurementMagneticField,
    MeasurementMagneticFieldErrorModel,
)
from aspn23.measurement_magnetic_field_magnitude import (
    MeasurementMagneticFieldMagnitude,
    MeasurementMagneticFieldMagnitudeErrorModel,
)
from aspn23.measurement_navsim_satnav_with_sv_data import (
    MeasurementNavsimSatnavWithSvData,
)
from aspn23.measurement_position import (
    MeasurementPosition,
    MeasurementPositionErrorModel,
    MeasurementPositionReferenceFrame,
)
from aspn23.measurement_position_attitude import (
    MeasurementPositionAttitude,
    MeasurementPositionAttitudeErrorModel,
    MeasurementPositionAttitudeReferenceFrame,
)
from aspn23.measurement_position_velocity_attitude import (
    MeasurementPositionVelocityAttitude,
    MeasurementPositionVelocityAttitudeErrorModel,
    MeasurementPositionVelocityAttitudeReferenceFrame,
)
from aspn23.measurement_range_rate_to_point import (
    MeasurementRangeRateToPoint,
    MeasurementRangeRateToPointErrorModel,
)
from aspn23.measurement_range_to_point import (
    MeasurementRangeToPoint,
    MeasurementRangeToPointErrorModel,
)
from aspn23.measurement_satnav import MeasurementSatnav
from aspn23.measurement_satnav_subframe import MeasurementSatnavSubframe
from aspn23.measurement_satnav_with_sv_data import MeasurementSatnavWithSvData
from aspn23.measurement_specific_force_1d import (
    MeasurementSpecificForce1D,
    MeasurementSpecificForce1DErrorModel,
    MeasurementSpecificForce1DSensorType,
)
from aspn23.measurement_speed import (
    MeasurementSpeed,
    MeasurementSpeedErrorModel,
    MeasurementSpeedReference,
)
from aspn23.measurement_TDOA_1Tx_2Rx import (
    MeasurementTdoa1Tx2Rx,
    MeasurementTdoa1Tx2RxErrorModel,
)
from aspn23.measurement_TDOA_2Tx_1Rx import (
    MeasurementTdoa2Tx1Rx,
    MeasurementTdoa2Tx1RxErrorModel,
)
from aspn23.measurement_temperature import (
    MeasurementTemperature,
    MeasurementTemperatureErrorModel,
)
from aspn23.measurement_time import MeasurementTime, MeasurementTimeErrorModel
from aspn23.measurement_time_difference import (
    MeasurementTimeDifference,
    MeasurementTimeDifferenceErrorModel,
)
from aspn23.measurement_time_frequency_difference import (
    MeasurementTimeFrequencyDifference,
    MeasurementTimeFrequencyDifferenceErrorModel,
)
from aspn23.measurement_velocity import (
    MeasurementVelocity,
    MeasurementVelocityErrorModel,
    MeasurementVelocityReferenceFrame,
)
from aspn23.metadata_BeiDou_ephemeris import MetadataBeidouEphemeris
from aspn23.metadata_Galileo_ephemeris import (
    MetadataGalileoEphemeris,
    MetadataGalileoEphemerisNavMsgType,
)
from aspn23.metadata_generic import MetadataGeneric
from aspn23.metadata_GLONASS_ephemeris import MetadataGlonassEphemeris
from aspn23.metadata_GPS_Cnav_ephemeris import MetadataGpsCnavEphemeris
from aspn23.metadata_GPS_iono_utc_parameters import MetadataGpsIonoUtcParameters
from aspn23.metadata_GPS_Lnav_ephemeris import MetadataGpsLnavEphemeris
from aspn23.metadata_GPS_Mnav_ephemeris import MetadataGpsMnavEphemeris
from aspn23.metadata_image_features import (
    MetadataImageFeatures,
    MetadataImageFeaturesDescriptorExtractor,
    MetadataImageFeaturesDescriptorType,
    MetadataImageFeaturesKeypointDetector,
)
from aspn23.metadata_IMU import MetadataImu, MetadataImuErrorModel
from aspn23.metadata_magnetic_field import MetadataMagneticField
from aspn23.metadata_satnav_obs import MetadataSatnavObs
from aspn23.type_direction_2d_to_point import (
    TypeDirection2DToPoint,
    TypeDirection2DToPointErrorModel,
    TypeDirection2DToPointReference,
)
from aspn23.type_direction_3d_to_point import (
    TypeDirection3DToPoint,
    TypeDirection3DToPointErrorModel,
    TypeDirection3DToPointReferenceFrame,
)
from aspn23.type_header import TypeHeader
from aspn23.type_image_feature import TypeImageFeature
from aspn23.type_integrity import TypeIntegrity, TypeIntegrityIntegrityMethod
from aspn23.type_kepler_orbit import TypeKeplerOrbit
from aspn23.type_metadataheader import TypeMetadataheader
from aspn23.type_mounting import TypeMounting
from aspn23.type_navsim_satnav_obs import (
    TypeNavsimSatnavObs,
    TypeNavsimSatnavObsIonoCorrectionSource,
    TypeNavsimSatnavObsPseudorangeRateType,
)
from aspn23.type_navsim_satnav_sv_data import (
    TypeNavsimSatnavSvData,
    TypeNavsimSatnavSvDataCoordinateFrame,
    TypeNavsimSatnavSvDataEphemerisType,
    TypeNavsimSatnavSvDataGroupDelayEnum,
)
from aspn23.type_remote_point import (
    TypeRemotePoint,
    TypeRemotePointPositionReferenceFrame,
)
from aspn23.type_satnav_clock import TypeSatnavClock
from aspn23.type_satnav_obs import (
    TypeSatnavObs,
    TypeSatnavObsIonoCorrectionSource,
    TypeSatnavObsPseudorangeRateType,
)
from aspn23.type_satnav_satellite_system import (
    TypeSatnavSatelliteSystem,
    TypeSatnavSatelliteSystemSatelliteSystem,
)
from aspn23.type_satnav_signal_descriptor import (
    TypeSatnavSignalDescriptor,
    TypeSatnavSignalDescriptorSignalDescriptor,
)
from aspn23.type_satnav_sv_data import (
    TypeSatnavSvData,
    TypeSatnavSvDataCoordinateFrame,
    TypeSatnavSvDataEphemerisType,
    TypeSatnavSvDataGroupDelayEnum,
)
from aspn23.type_satnav_time import TypeSatnavTime, TypeSatnavTimeTimeReference
from aspn23.type_timestamp import TypeTimestamp

from .image import image as LcmImage
from .measurement_accumulated_distance_traveled import (
    measurement_accumulated_distance_traveled as LcmMeasurementAccumulatedDistanceTraveled,
)
from .measurement_altitude import measurement_altitude as LcmMeasurementAltitude
from .measurement_angular_velocity import (
    measurement_angular_velocity as LcmMeasurementAngularVelocity,
)
from .measurement_angular_velocity_1d import (
    measurement_angular_velocity_1d as LcmMeasurementAngularVelocity1D,
)
from .measurement_attitude_2d import measurement_attitude_2d as LcmMeasurementAttitude2D
from .measurement_attitude_3d import measurement_attitude_3d as LcmMeasurementAttitude3D
from .measurement_barometer import measurement_barometer as LcmMeasurementBarometer
from .measurement_delta_position import (
    measurement_delta_position as LcmMeasurementDeltaPosition,
)
from .measurement_delta_range import measurement_delta_range as LcmMeasurementDeltaRange
from .measurement_delta_range_to_point import (
    measurement_delta_range_to_point as LcmMeasurementDeltaRangeToPoint,
)
from .measurement_direction_2d_to_points import (
    measurement_direction_2d_to_points as LcmMeasurementDirection2DToPoints,
)
from .measurement_direction_3d_to_points import (
    measurement_direction_3d_to_points as LcmMeasurementDirection3DToPoints,
)
from .measurement_direction_of_motion_2d import (
    measurement_direction_of_motion_2d as LcmMeasurementDirectionOfMotion2D,
)
from .measurement_direction_of_motion_3d import (
    measurement_direction_of_motion_3d as LcmMeasurementDirectionOfMotion3D,
)
from .measurement_frequency_difference import (
    measurement_frequency_difference as LcmMeasurementFrequencyDifference,
)
from .measurement_heading import measurement_heading as LcmMeasurementHeading
from .measurement_IMU import measurement_IMU as LcmMeasurementImu
from .measurement_magnetic_field import (
    measurement_magnetic_field as LcmMeasurementMagneticField,
)
from .measurement_magnetic_field_magnitude import (
    measurement_magnetic_field_magnitude as LcmMeasurementMagneticFieldMagnitude,
)
from .measurement_navsim_satnav_with_sv_data import (
    measurement_navsim_satnav_with_sv_data as LcmMeasurementNavsimSatnavWithSvData,
)
from .measurement_position import measurement_position as LcmMeasurementPosition
from .measurement_position_attitude import (
    measurement_position_attitude as LcmMeasurementPositionAttitude,
)
from .measurement_position_velocity_attitude import (
    measurement_position_velocity_attitude as LcmMeasurementPositionVelocityAttitude,
)
from .measurement_range_rate_to_point import (
    measurement_range_rate_to_point as LcmMeasurementRangeRateToPoint,
)
from .measurement_range_to_point import (
    measurement_range_to_point as LcmMeasurementRangeToPoint,
)
from .measurement_satnav import measurement_satnav as LcmMeasurementSatnav
from .measurement_satnav_subframe import (
    measurement_satnav_subframe as LcmMeasurementSatnavSubframe,
)
from .measurement_satnav_with_sv_data import (
    measurement_satnav_with_sv_data as LcmMeasurementSatnavWithSvData,
)
from .measurement_specific_force_1d import (
    measurement_specific_force_1d as LcmMeasurementSpecificForce1D,
)
from .measurement_speed import measurement_speed as LcmMeasurementSpeed
from .measurement_TDOA_1Tx_2Rx import (
    measurement_TDOA_1Tx_2Rx as LcmMeasurementTdoa1Tx2Rx,
)
from .measurement_TDOA_2Tx_1Rx import (
    measurement_TDOA_2Tx_1Rx as LcmMeasurementTdoa2Tx1Rx,
)
from .measurement_temperature import (
    measurement_temperature as LcmMeasurementTemperature,
)
from .measurement_time import measurement_time as LcmMeasurementTime
from .measurement_time_difference import (
    measurement_time_difference as LcmMeasurementTimeDifference,
)
from .measurement_time_frequency_difference import (
    measurement_time_frequency_difference as LcmMeasurementTimeFrequencyDifference,
)
from .measurement_velocity import measurement_velocity as LcmMeasurementVelocity
from .metadata_BeiDou_ephemeris import (
    metadata_BeiDou_ephemeris as LcmMetadataBeidouEphemeris,
)
from .metadata_Galileo_ephemeris import (
    metadata_Galileo_ephemeris as LcmMetadataGalileoEphemeris,
)
from .metadata_generic import metadata_generic as LcmMetadataGeneric
from .metadata_GLONASS_ephemeris import (
    metadata_GLONASS_ephemeris as LcmMetadataGlonassEphemeris,
)
from .metadata_GPS_Cnav_ephemeris import (
    metadata_GPS_Cnav_ephemeris as LcmMetadataGpsCnavEphemeris,
)
from .metadata_GPS_iono_utc_parameters import (
    metadata_GPS_iono_utc_parameters as LcmMetadataGpsIonoUtcParameters,
)
from .metadata_GPS_Lnav_ephemeris import (
    metadata_GPS_Lnav_ephemeris as LcmMetadataGpsLnavEphemeris,
)
from .metadata_GPS_Mnav_ephemeris import (
    metadata_GPS_Mnav_ephemeris as LcmMetadataGpsMnavEphemeris,
)
from .metadata_image_features import metadata_image_features as LcmMetadataImageFeatures
from .metadata_IMU import metadata_IMU as LcmMetadataImu
from .metadata_magnetic_field import metadata_magnetic_field as LcmMetadataMagneticField
from .metadata_satnav_obs import metadata_satnav_obs as LcmMetadataSatnavObs
from .type_direction_2d_to_point import (
    type_direction_2d_to_point as LcmTypeDirection2DToPoint,
)
from .type_direction_3d_to_point import (
    type_direction_3d_to_point as LcmTypeDirection3DToPoint,
)
from .type_header import type_header as LcmTypeHeader
from .type_image_feature import type_image_feature as LcmTypeImageFeature
from .type_integrity import type_integrity as LcmTypeIntegrity
from .type_kepler_orbit import type_kepler_orbit as LcmTypeKeplerOrbit
from .type_metadataheader import type_metadataheader as LcmTypeMetadataheader
from .type_mounting import type_mounting as LcmTypeMounting
from .type_navsim_satnav_obs import type_navsim_satnav_obs as LcmTypeNavsimSatnavObs
from .type_navsim_satnav_sv_data import (
    type_navsim_satnav_sv_data as LcmTypeNavsimSatnavSvData,
)
from .type_remote_point import type_remote_point as LcmTypeRemotePoint
from .type_satnav_clock import type_satnav_clock as LcmTypeSatnavClock
from .type_satnav_obs import type_satnav_obs as LcmTypeSatnavObs
from .type_satnav_satellite_system import (
    type_satnav_satellite_system as LcmTypeSatnavSatelliteSystem,
)
from .type_satnav_signal_descriptor import (
    type_satnav_signal_descriptor as LcmTypeSatnavSignalDescriptor,
)
from .type_satnav_sv_data import type_satnav_sv_data as LcmTypeSatnavSvData
from .type_satnav_time import type_satnav_time as LcmTypeSatnavTime
from .type_timestamp import type_timestamp as LcmTypeTimestamp


def type_direction_2d_to_point_to_lcm(
    old: TypeDirection2DToPoint,
) -> LcmTypeDirection2DToPoint:
    msg = LcmTypeDirection2DToPoint()
    msg.remote_point = type_remote_point_to_lcm(old.remote_point)
    msg.reference = old.reference.value
    msg.obs = old.obs
    msg.variance = old.variance
    msg.has_observation_characteristics = old.has_observation_characteristics
    msg.observation_characteristics = type_image_feature_to_lcm(
        old.observation_characteristics
    )
    msg.error_model = old.error_model.value
    msg.error_model_params = old.error_model_params.tolist()
    msg.num_error_model_params = len(old.error_model_params)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_type_direction_2d_to_point(
    old: LcmTypeDirection2DToPoint,
) -> TypeDirection2DToPoint:
    return TypeDirection2DToPoint(
        remote_point=lcm_to_type_remote_point(old.remote_point),
        reference=TypeDirection2DToPointReference(old.reference),
        obs=old.obs,
        variance=old.variance,
        has_observation_characteristics=old.has_observation_characteristics,
        observation_characteristics=lcm_to_type_image_feature(
            old.observation_characteristics
        ),
        error_model=TypeDirection2DToPointErrorModel(old.error_model),
        error_model_params=np.array(old.error_model_params),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def type_direction_3d_to_point_to_lcm(
    old: TypeDirection3DToPoint,
) -> LcmTypeDirection3DToPoint:
    msg = LcmTypeDirection3DToPoint()
    msg.remote_point = type_remote_point_to_lcm(old.remote_point)
    msg.reference_frame = old.reference_frame.value
    msg.obs = old.obs.tolist()
    msg.covariance = old.covariance.tolist()
    msg.has_observation_characteristics = old.has_observation_characteristics
    msg.observation_characteristics = type_image_feature_to_lcm(
        old.observation_characteristics
    )
    msg.error_model = old.error_model.value
    msg.error_model_params = old.error_model_params.tolist()
    msg.num_error_model_params = len(old.error_model_params)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_type_direction_3d_to_point(
    old: LcmTypeDirection3DToPoint,
) -> TypeDirection3DToPoint:
    return TypeDirection3DToPoint(
        remote_point=lcm_to_type_remote_point(old.remote_point),
        reference_frame=TypeDirection3DToPointReferenceFrame(old.reference_frame),
        obs=np.array(old.obs),
        covariance=np.array(old.covariance),
        has_observation_characteristics=old.has_observation_characteristics,
        observation_characteristics=lcm_to_type_image_feature(
            old.observation_characteristics
        ),
        error_model=TypeDirection3DToPointErrorModel(old.error_model),
        error_model_params=np.array(old.error_model_params),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def type_header_to_lcm(old: TypeHeader) -> LcmTypeHeader:
    msg = LcmTypeHeader()
    msg.vendor_id = old.vendor_id
    msg.device_id = old.device_id
    msg.context_id = old.context_id
    msg.sequence_id = old.sequence_id

    return msg


def lcm_to_type_header(old: LcmTypeHeader) -> TypeHeader:
    return TypeHeader(
        vendor_id=old.vendor_id,
        device_id=old.device_id,
        context_id=old.context_id,
        sequence_id=old.sequence_id,
    )


def type_image_feature_to_lcm(old: TypeImageFeature) -> LcmTypeImageFeature:
    msg = LcmTypeImageFeature()
    msg.response = old.response
    msg.orientation = old.orientation
    msg.size = old.size
    msg.class_id = old.class_id
    msg.octave = old.octave
    msg.descriptor = old.descriptor.tolist()
    msg.descriptor_size = len(old.descriptor)

    return msg


def lcm_to_type_image_feature(old: LcmTypeImageFeature) -> TypeImageFeature:
    return TypeImageFeature(
        response=old.response,
        orientation=old.orientation,
        size=old.size,
        class_id=old.class_id,
        octave=old.octave,
        descriptor=np.array(old.descriptor),
    )


def type_integrity_to_lcm(old: TypeIntegrity) -> LcmTypeIntegrity:
    msg = LcmTypeIntegrity()
    msg.integrity_method = old.integrity_method.value
    msg.integrity_value = (
        old.integrity_value if old.integrity_value is not None else float()
    )

    return msg


def lcm_to_type_integrity(old: LcmTypeIntegrity) -> TypeIntegrity:
    return TypeIntegrity(
        integrity_method=TypeIntegrityIntegrityMethod(old.integrity_method),
        integrity_value=old.integrity_value,
    )


def type_kepler_orbit_to_lcm(old: TypeKeplerOrbit) -> LcmTypeKeplerOrbit:
    msg = LcmTypeKeplerOrbit()
    msg.m_0 = old.m_0
    msg.delta_n = old.delta_n
    msg.e = old.e
    msg.sqrt_a = old.sqrt_a
    msg.omega_0 = old.omega_0
    msg.i_0 = old.i_0
    msg.i_dot = old.i_dot
    msg.omega = old.omega
    msg.omega_dot = old.omega_dot
    msg.c_uc = old.c_uc
    msg.c_us = old.c_us
    msg.c_rc = old.c_rc
    msg.c_rs = old.c_rs
    msg.c_ic = old.c_ic
    msg.c_is = old.c_is
    msg.t_oe = old.t_oe

    return msg


def lcm_to_type_kepler_orbit(old: LcmTypeKeplerOrbit) -> TypeKeplerOrbit:
    return TypeKeplerOrbit(
        m_0=old.m_0,
        delta_n=old.delta_n,
        e=old.e,
        sqrt_a=old.sqrt_a,
        omega_0=old.omega_0,
        i_0=old.i_0,
        i_dot=old.i_dot,
        omega=old.omega,
        omega_dot=old.omega_dot,
        c_uc=old.c_uc,
        c_us=old.c_us,
        c_rc=old.c_rc,
        c_rs=old.c_rs,
        c_ic=old.c_ic,
        c_is=old.c_is,
        t_oe=old.t_oe,
    )


def type_metadataheader_to_lcm(old: TypeMetadataheader) -> LcmTypeMetadataheader:
    msg = LcmTypeMetadataheader()
    msg.header = type_header_to_lcm(old.header)
    msg.sensor_description = old.sensor_description
    msg.delta_t_nom = old.delta_t_nom if old.delta_t_nom is not None else float()
    msg.timestamp_clock_id = old.timestamp_clock_id
    msg.digits_of_precision = old.digits_of_precision

    return msg


def lcm_to_type_metadataheader(old: LcmTypeMetadataheader) -> TypeMetadataheader:
    return TypeMetadataheader(
        header=lcm_to_type_header(old.header),
        sensor_description=old.sensor_description,
        delta_t_nom=old.delta_t_nom,
        timestamp_clock_id=old.timestamp_clock_id,
        digits_of_precision=old.digits_of_precision,
    )


def type_mounting_to_lcm(old: TypeMounting) -> LcmTypeMounting:
    msg = LcmTypeMounting()
    msg.lever_arm = old.lever_arm.tolist()
    msg.lever_arm_sigma = old.lever_arm_sigma.tolist()
    msg.orientation_quaternion = (
        old.orientation_quaternion.tolist()
        if old.orientation_quaternion is not None
        else []
    )
    msg.orientation_tilt_error_covariance = (
        old.orientation_tilt_error_covariance.tolist()
        if old.orientation_tilt_error_covariance is not None
        else []
    )

    return msg


def lcm_to_type_mounting(old: LcmTypeMounting) -> TypeMounting:
    return TypeMounting(
        lever_arm=np.array(old.lever_arm),
        lever_arm_sigma=np.array(old.lever_arm_sigma),
        orientation_quaternion=np.array(old.orientation_quaternion),
        orientation_tilt_error_covariance=np.array(
            old.orientation_tilt_error_covariance
        ),
    )


def type_navsim_satnav_obs_to_lcm(old: TypeNavsimSatnavObs) -> LcmTypeNavsimSatnavObs:
    msg = LcmTypeNavsimSatnavObs()
    msg.satellite_system = old.satellite_system
    msg.signal_descriptor = old.signal_descriptor
    msg.prn = old.prn
    msg.frequency = old.frequency
    msg.pseudorange = old.pseudorange if old.pseudorange is not None else float()
    msg.pseudorange_variance = (
        old.pseudorange_variance if old.pseudorange_variance is not None else float()
    )
    msg.pseudorange_rate_type = old.pseudorange_rate_type.value
    msg.pseudorange_rate = (
        old.pseudorange_rate if old.pseudorange_rate is not None else float()
    )
    msg.pseudorange_rate_variance = (
        old.pseudorange_rate_variance
        if old.pseudorange_rate_variance is not None
        else float()
    )
    msg.carrier_phase = old.carrier_phase if old.carrier_phase is not None else float()
    msg.carrier_phase_variance = (
        old.carrier_phase_variance
        if old.carrier_phase_variance is not None
        else float()
    )
    msg.c_n0 = old.c_n0 if old.c_n0 is not None else float()
    msg.lock_count = old.lock_count
    msg.iono_correction_source = old.iono_correction_source.value
    msg.iono_correction_applied = old.iono_correction_applied
    msg.tropo_correction_applied = old.tropo_correction_applied
    msg.signal_bias_correction_applied = old.signal_bias_correction_applied
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_type_navsim_satnav_obs(old: LcmTypeNavsimSatnavObs) -> TypeNavsimSatnavObs:
    return TypeNavsimSatnavObs(
        satellite_system=old.satellite_system,
        signal_descriptor=old.signal_descriptor,
        prn=old.prn,
        frequency=old.frequency,
        pseudorange=old.pseudorange,
        pseudorange_variance=old.pseudorange_variance,
        pseudorange_rate_type=TypeNavsimSatnavObsPseudorangeRateType(
            old.pseudorange_rate_type
        ),
        pseudorange_rate=old.pseudorange_rate,
        pseudorange_rate_variance=old.pseudorange_rate_variance,
        carrier_phase=old.carrier_phase,
        carrier_phase_variance=old.carrier_phase_variance,
        c_n0=old.c_n0,
        lock_count=old.lock_count,
        iono_correction_source=TypeNavsimSatnavObsIonoCorrectionSource(
            old.iono_correction_source
        ),
        iono_correction_applied=old.iono_correction_applied,
        tropo_correction_applied=old.tropo_correction_applied,
        signal_bias_correction_applied=old.signal_bias_correction_applied,
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def type_navsim_satnav_sv_data_to_lcm(
    old: TypeNavsimSatnavSvData,
) -> LcmTypeNavsimSatnavSvData:
    msg = LcmTypeNavsimSatnavSvData()
    msg.prn = old.prn
    msg.satellite_system = old.satellite_system
    msg.ephemeris_type = old.ephemeris_type.value
    msg.sv_data_time = type_satnav_time_to_lcm(old.sv_data_time)
    msg.coordinate_frame = old.coordinate_frame.value
    msg.sv_pos = old.sv_pos.tolist()
    msg.sv_vel = old.sv_vel.tolist()
    msg.sv_clock_bias = old.sv_clock_bias
    msg.sv_clock_drift = old.sv_clock_drift
    msg.group_delay_enum = old.group_delay_enum.value
    msg.group_delay_vector = old.group_delay_vector.tolist()

    return msg


def lcm_to_type_navsim_satnav_sv_data(
    old: LcmTypeNavsimSatnavSvData,
) -> TypeNavsimSatnavSvData:
    return TypeNavsimSatnavSvData(
        prn=old.prn,
        satellite_system=old.satellite_system,
        ephemeris_type=TypeNavsimSatnavSvDataEphemerisType(old.ephemeris_type),
        sv_data_time=lcm_to_type_satnav_time(old.sv_data_time),
        coordinate_frame=TypeNavsimSatnavSvDataCoordinateFrame(old.coordinate_frame),
        sv_pos=np.array(old.sv_pos),
        sv_vel=np.array(old.sv_vel),
        sv_clock_bias=old.sv_clock_bias,
        sv_clock_drift=old.sv_clock_drift,
        group_delay_enum=TypeNavsimSatnavSvDataGroupDelayEnum(old.group_delay_enum),
        group_delay_vector=np.array(old.group_delay_vector),
    )


def type_remote_point_to_lcm(old: TypeRemotePoint) -> LcmTypeRemotePoint:
    msg = LcmTypeRemotePoint()
    msg.included_terms = old.included_terms
    msg.id = old.id
    msg.position_reference_frame = old.position_reference_frame.value
    msg.position1 = old.position1 if old.position1 is not None else float()
    msg.position2 = old.position2 if old.position2 is not None else float()
    msg.position3 = old.position3 if old.position3 is not None else float()
    msg.position_covariance = old.position_covariance.tolist()
    msg.num_position_components = len(old.position_covariance)

    return msg


def lcm_to_type_remote_point(old: LcmTypeRemotePoint) -> TypeRemotePoint:
    return TypeRemotePoint(
        included_terms=old.included_terms,
        id=old.id,
        position_reference_frame=TypeRemotePointPositionReferenceFrame(
            old.position_reference_frame
        ),
        position1=old.position1,
        position2=old.position2,
        position3=old.position3,
        position_covariance=np.array(old.position_covariance),
    )


def type_satnav_clock_to_lcm(old: TypeSatnavClock) -> LcmTypeSatnavClock:
    msg = LcmTypeSatnavClock()
    msg.t_oc = old.t_oc
    msg.af_0 = old.af_0
    msg.af_1 = old.af_1
    msg.af_2 = old.af_2

    return msg


def lcm_to_type_satnav_clock(old: LcmTypeSatnavClock) -> TypeSatnavClock:
    return TypeSatnavClock(t_oc=old.t_oc, af_0=old.af_0, af_1=old.af_1, af_2=old.af_2)


def type_satnav_obs_to_lcm(old: TypeSatnavObs) -> LcmTypeSatnavObs:
    msg = LcmTypeSatnavObs()
    msg.satellite_system = type_satnav_satellite_system_to_lcm(old.satellite_system)
    msg.signal_descriptor = type_satnav_signal_descriptor_to_lcm(old.signal_descriptor)
    msg.prn = old.prn
    msg.frequency = old.frequency
    msg.pseudorange = old.pseudorange if old.pseudorange is not None else float()
    msg.pseudorange_variance = (
        old.pseudorange_variance if old.pseudorange_variance is not None else float()
    )
    msg.pseudorange_rate_type = old.pseudorange_rate_type.value
    msg.pseudorange_rate = (
        old.pseudorange_rate if old.pseudorange_rate is not None else float()
    )
    msg.pseudorange_rate_variance = (
        old.pseudorange_rate_variance
        if old.pseudorange_rate_variance is not None
        else float()
    )
    msg.carrier_phase = old.carrier_phase if old.carrier_phase is not None else float()
    msg.carrier_phase_variance = (
        old.carrier_phase_variance
        if old.carrier_phase_variance is not None
        else float()
    )
    msg.c_n0 = old.c_n0 if old.c_n0 is not None else float()
    msg.lock_count = old.lock_count
    msg.iono_correction_source = old.iono_correction_source.value
    msg.iono_correction_applied = old.iono_correction_applied
    msg.tropo_correction_applied = old.tropo_correction_applied
    msg.signal_bias_correction_applied = old.signal_bias_correction_applied
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_type_satnav_obs(old: LcmTypeSatnavObs) -> TypeSatnavObs:
    return TypeSatnavObs(
        satellite_system=lcm_to_type_satnav_satellite_system(old.satellite_system),
        signal_descriptor=lcm_to_type_satnav_signal_descriptor(old.signal_descriptor),
        prn=old.prn,
        frequency=old.frequency,
        pseudorange=old.pseudorange,
        pseudorange_variance=old.pseudorange_variance,
        pseudorange_rate_type=TypeSatnavObsPseudorangeRateType(
            old.pseudorange_rate_type
        ),
        pseudorange_rate=old.pseudorange_rate,
        pseudorange_rate_variance=old.pseudorange_rate_variance,
        carrier_phase=old.carrier_phase,
        carrier_phase_variance=old.carrier_phase_variance,
        c_n0=old.c_n0,
        lock_count=old.lock_count,
        iono_correction_source=TypeSatnavObsIonoCorrectionSource(
            old.iono_correction_source
        ),
        iono_correction_applied=old.iono_correction_applied,
        tropo_correction_applied=old.tropo_correction_applied,
        signal_bias_correction_applied=old.signal_bias_correction_applied,
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def type_satnav_satellite_system_to_lcm(
    old: TypeSatnavSatelliteSystem,
) -> LcmTypeSatnavSatelliteSystem:
    msg = LcmTypeSatnavSatelliteSystem()
    msg.satellite_system = old.satellite_system.value

    return msg


def lcm_to_type_satnav_satellite_system(
    old: LcmTypeSatnavSatelliteSystem,
) -> TypeSatnavSatelliteSystem:
    return TypeSatnavSatelliteSystem(
        satellite_system=TypeSatnavSatelliteSystemSatelliteSystem(old.satellite_system)
    )


def type_satnav_signal_descriptor_to_lcm(
    old: TypeSatnavSignalDescriptor,
) -> LcmTypeSatnavSignalDescriptor:
    msg = LcmTypeSatnavSignalDescriptor()
    msg.signal_descriptor = old.signal_descriptor.value

    return msg


def lcm_to_type_satnav_signal_descriptor(
    old: LcmTypeSatnavSignalDescriptor,
) -> TypeSatnavSignalDescriptor:
    return TypeSatnavSignalDescriptor(
        signal_descriptor=TypeSatnavSignalDescriptorSignalDescriptor(
            old.signal_descriptor
        )
    )


def type_satnav_sv_data_to_lcm(old: TypeSatnavSvData) -> LcmTypeSatnavSvData:
    msg = LcmTypeSatnavSvData()
    msg.prn = old.prn
    msg.satellite_system = type_satnav_satellite_system_to_lcm(old.satellite_system)
    msg.ephemeris_type = old.ephemeris_type.value
    msg.sv_data_time = type_satnav_time_to_lcm(old.sv_data_time)
    msg.coordinate_frame = old.coordinate_frame.value
    msg.sv_pos = old.sv_pos.tolist()
    msg.sv_vel = old.sv_vel.tolist()
    msg.sv_clock_bias = old.sv_clock_bias
    msg.sv_clock_drift = old.sv_clock_drift
    msg.group_delay_enum = old.group_delay_enum.value
    msg.group_delay_vector = old.group_delay_vector.tolist()

    return msg


def lcm_to_type_satnav_sv_data(old: LcmTypeSatnavSvData) -> TypeSatnavSvData:
    return TypeSatnavSvData(
        prn=old.prn,
        satellite_system=lcm_to_type_satnav_satellite_system(old.satellite_system),
        ephemeris_type=TypeSatnavSvDataEphemerisType(old.ephemeris_type),
        sv_data_time=lcm_to_type_satnav_time(old.sv_data_time),
        coordinate_frame=TypeSatnavSvDataCoordinateFrame(old.coordinate_frame),
        sv_pos=np.array(old.sv_pos),
        sv_vel=np.array(old.sv_vel),
        sv_clock_bias=old.sv_clock_bias,
        sv_clock_drift=old.sv_clock_drift,
        group_delay_enum=TypeSatnavSvDataGroupDelayEnum(old.group_delay_enum),
        group_delay_vector=np.array(old.group_delay_vector),
    )


def type_satnav_time_to_lcm(old: TypeSatnavTime) -> LcmTypeSatnavTime:
    msg = LcmTypeSatnavTime()
    msg.week_number = old.week_number
    msg.seconds_of_week = old.seconds_of_week
    msg.time_reference = old.time_reference.value

    return msg


def lcm_to_type_satnav_time(old: LcmTypeSatnavTime) -> TypeSatnavTime:
    return TypeSatnavTime(
        week_number=old.week_number,
        seconds_of_week=old.seconds_of_week,
        time_reference=TypeSatnavTimeTimeReference(old.time_reference),
    )


def type_timestamp_to_lcm(old: TypeTimestamp) -> LcmTypeTimestamp:
    msg = LcmTypeTimestamp()
    msg.elapsed_nsec = old.elapsed_nsec

    return msg


def lcm_to_type_timestamp(old: LcmTypeTimestamp) -> TypeTimestamp:
    return TypeTimestamp(elapsed_nsec=old.elapsed_nsec)


def metadata_BeiDou_ephemeris_to_lcm(
    old: MetadataBeidouEphemeris,
) -> LcmMetadataBeidouEphemeris:
    msg = LcmMetadataBeidouEphemeris()
    msg.info = type_metadataheader_to_lcm(old.info)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.prn = old.prn
    msg.clock = type_satnav_clock_to_lcm(old.clock)
    msg.orbit = type_kepler_orbit_to_lcm(old.orbit)
    msg.t_gd1 = old.t_gd1
    msg.t_gd2 = old.t_gd2
    msg.aodc = old.aodc
    msg.aode = old.aode

    return msg


def lcm_to_metadata_BeiDou_ephemeris(
    old: LcmMetadataBeidouEphemeris,
) -> MetadataBeidouEphemeris:
    return MetadataBeidouEphemeris(
        info=lcm_to_type_metadataheader(old.info),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        prn=old.prn,
        clock=lcm_to_type_satnav_clock(old.clock),
        orbit=lcm_to_type_kepler_orbit(old.orbit),
        t_gd1=old.t_gd1,
        t_gd2=old.t_gd2,
        aodc=old.aodc,
        aode=old.aode,
    )


def metadata_GLONASS_ephemeris_to_lcm(
    old: MetadataGlonassEphemeris,
) -> LcmMetadataGlonassEphemeris:
    msg = LcmMetadataGlonassEphemeris()
    msg.info = type_metadataheader_to_lcm(old.info)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.slot_number = old.slot_number
    msg.freq_o = old.freq_o
    msg.n_t = old.n_t
    msg.t_k = old.t_k
    msg.t_b = old.t_b
    msg.gamma_n = old.gamma_n
    msg.tau_n = old.tau_n
    msg.sv_pos_x = old.sv_pos_x
    msg.sv_vel_x = old.sv_vel_x
    msg.sv_accel_x = old.sv_accel_x
    msg.sv_pos_y = old.sv_pos_y
    msg.sv_vel_y = old.sv_vel_y
    msg.sv_accel_y = old.sv_accel_y
    msg.sv_pos_z = old.sv_pos_z
    msg.sv_vel_z = old.sv_vel_z
    msg.sv_accel_z = old.sv_accel_z

    return msg


def lcm_to_metadata_GLONASS_ephemeris(
    old: LcmMetadataGlonassEphemeris,
) -> MetadataGlonassEphemeris:
    return MetadataGlonassEphemeris(
        info=lcm_to_type_metadataheader(old.info),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        slot_number=old.slot_number,
        freq_o=old.freq_o,
        n_t=old.n_t,
        t_k=old.t_k,
        t_b=old.t_b,
        gamma_n=old.gamma_n,
        tau_n=old.tau_n,
        sv_pos_x=old.sv_pos_x,
        sv_vel_x=old.sv_vel_x,
        sv_accel_x=old.sv_accel_x,
        sv_pos_y=old.sv_pos_y,
        sv_vel_y=old.sv_vel_y,
        sv_accel_y=old.sv_accel_y,
        sv_pos_z=old.sv_pos_z,
        sv_vel_z=old.sv_vel_z,
        sv_accel_z=old.sv_accel_z,
    )


def metadata_GPS_Cnav_ephemeris_to_lcm(
    old: MetadataGpsCnavEphemeris,
) -> LcmMetadataGpsCnavEphemeris:
    msg = LcmMetadataGpsCnavEphemeris()
    msg.info = type_metadataheader_to_lcm(old.info)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.week_number = old.week_number if old.week_number is not None else int()
    msg.prn = old.prn
    msg.clock = type_satnav_clock_to_lcm(old.clock)
    msg.orbit = type_kepler_orbit_to_lcm(old.orbit)
    msg.t_gd = old.t_gd
    msg.iodc = old.iodc
    msg.iode = old.iode
    msg.isc_l1_ca = old.isc_l1_ca
    msg.isc_l2_c = old.isc_l2_c
    msg.isc_l5_i5 = old.isc_l5_i5
    msg.isc_l5_q5 = old.isc_l5_q5
    msg.delta_a_0 = old.delta_a_0
    msg.a_dot = old.a_dot

    return msg


def lcm_to_metadata_GPS_Cnav_ephemeris(
    old: LcmMetadataGpsCnavEphemeris,
) -> MetadataGpsCnavEphemeris:
    return MetadataGpsCnavEphemeris(
        info=lcm_to_type_metadataheader(old.info),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        week_number=old.week_number,
        prn=old.prn,
        clock=lcm_to_type_satnav_clock(old.clock),
        orbit=lcm_to_type_kepler_orbit(old.orbit),
        t_gd=old.t_gd,
        iodc=old.iodc,
        iode=old.iode,
        isc_l1_ca=old.isc_l1_ca,
        isc_l2_c=old.isc_l2_c,
        isc_l5_i5=old.isc_l5_i5,
        isc_l5_q5=old.isc_l5_q5,
        delta_a_0=old.delta_a_0,
        a_dot=old.a_dot,
    )


def metadata_GPS_Lnav_ephemeris_to_lcm(
    old: MetadataGpsLnavEphemeris,
) -> LcmMetadataGpsLnavEphemeris:
    msg = LcmMetadataGpsLnavEphemeris()
    msg.info = type_metadataheader_to_lcm(old.info)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.week_number = old.week_number if old.week_number is not None else int()
    msg.prn = old.prn
    msg.clock = type_satnav_clock_to_lcm(old.clock)
    msg.orbit = type_kepler_orbit_to_lcm(old.orbit)
    msg.t_gd = old.t_gd
    msg.iodc = old.iodc
    msg.iode = old.iode

    return msg


def lcm_to_metadata_GPS_Lnav_ephemeris(
    old: LcmMetadataGpsLnavEphemeris,
) -> MetadataGpsLnavEphemeris:
    return MetadataGpsLnavEphemeris(
        info=lcm_to_type_metadataheader(old.info),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        week_number=old.week_number,
        prn=old.prn,
        clock=lcm_to_type_satnav_clock(old.clock),
        orbit=lcm_to_type_kepler_orbit(old.orbit),
        t_gd=old.t_gd,
        iodc=old.iodc,
        iode=old.iode,
    )


def metadata_GPS_Mnav_ephemeris_to_lcm(
    old: MetadataGpsMnavEphemeris,
) -> LcmMetadataGpsMnavEphemeris:
    msg = LcmMetadataGpsMnavEphemeris()
    msg.info = type_metadataheader_to_lcm(old.info)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.week_number = old.week_number if old.week_number is not None else int()
    msg.prn = old.prn
    msg.clock = type_satnav_clock_to_lcm(old.clock)
    msg.orbit = type_kepler_orbit_to_lcm(old.orbit)
    msg.a_dot = old.a_dot
    msg.delta_af_0 = old.delta_af_0
    msg.delta_af_1 = old.delta_af_1
    msg.delta_gamma = old.delta_gamma
    msg.delta_i = old.delta_i
    msg.delta_omega = old.delta_omega
    msg.delta_a = old.delta_a
    msg.isc_l1_m_e = old.isc_l1_m_e
    msg.isc_l2_m_e = old.isc_l2_m_e
    msg.isc_l1_m_s = old.isc_l1_m_s
    msg.isc_l2_m_s = old.isc_l2_m_s
    msg.isa_l2_py = old.isa_l2_py
    msg.isa_l1_m_e = old.isa_l1_m_e
    msg.isa_l2_m_e = old.isa_l2_m_e
    msg.isa_l1_m_s = old.isa_l1_m_s
    msg.isa_l2_m_s = old.isa_l2_m_s

    return msg


def lcm_to_metadata_GPS_Mnav_ephemeris(
    old: LcmMetadataGpsMnavEphemeris,
) -> MetadataGpsMnavEphemeris:
    return MetadataGpsMnavEphemeris(
        info=lcm_to_type_metadataheader(old.info),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        week_number=old.week_number,
        prn=old.prn,
        clock=lcm_to_type_satnav_clock(old.clock),
        orbit=lcm_to_type_kepler_orbit(old.orbit),
        a_dot=old.a_dot,
        delta_af_0=old.delta_af_0,
        delta_af_1=old.delta_af_1,
        delta_gamma=old.delta_gamma,
        delta_i=old.delta_i,
        delta_omega=old.delta_omega,
        delta_a=old.delta_a,
        isc_l1_m_e=old.isc_l1_m_e,
        isc_l2_m_e=old.isc_l2_m_e,
        isc_l1_m_s=old.isc_l1_m_s,
        isc_l2_m_s=old.isc_l2_m_s,
        isa_l2_py=old.isa_l2_py,
        isa_l1_m_e=old.isa_l1_m_e,
        isa_l2_m_e=old.isa_l2_m_e,
        isa_l1_m_s=old.isa_l1_m_s,
        isa_l2_m_s=old.isa_l2_m_s,
    )


def metadata_GPS_iono_utc_parameters_to_lcm(
    old: MetadataGpsIonoUtcParameters,
) -> LcmMetadataGpsIonoUtcParameters:
    msg = LcmMetadataGpsIonoUtcParameters()
    msg.info = type_metadataheader_to_lcm(old.info)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.a_0 = old.a_0
    msg.a_1 = old.a_1
    msg.delta_t_ls = old.delta_t_ls
    msg.tot = old.tot
    msg.wn_t = old.wn_t
    msg.wn_lsf = old.wn_lsf
    msg.dn = old.dn
    msg.delta_t_lsf = old.delta_t_lsf
    msg.alpha_0 = old.alpha_0
    msg.alpha_1 = old.alpha_1
    msg.alpha_2 = old.alpha_2
    msg.alpha_3 = old.alpha_3
    msg.beta_0 = old.beta_0
    msg.beta_1 = old.beta_1
    msg.beta_2 = old.beta_2
    msg.beta_3 = old.beta_3

    return msg


def lcm_to_metadata_GPS_iono_utc_parameters(
    old: LcmMetadataGpsIonoUtcParameters,
) -> MetadataGpsIonoUtcParameters:
    return MetadataGpsIonoUtcParameters(
        info=lcm_to_type_metadataheader(old.info),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        a_0=old.a_0,
        a_1=old.a_1,
        delta_t_ls=old.delta_t_ls,
        tot=old.tot,
        wn_t=old.wn_t,
        wn_lsf=old.wn_lsf,
        dn=old.dn,
        delta_t_lsf=old.delta_t_lsf,
        alpha_0=old.alpha_0,
        alpha_1=old.alpha_1,
        alpha_2=old.alpha_2,
        alpha_3=old.alpha_3,
        beta_0=old.beta_0,
        beta_1=old.beta_1,
        beta_2=old.beta_2,
        beta_3=old.beta_3,
    )


def metadata_Galileo_ephemeris_to_lcm(
    old: MetadataGalileoEphemeris,
) -> LcmMetadataGalileoEphemeris:
    msg = LcmMetadataGalileoEphemeris()
    msg.info = type_metadataheader_to_lcm(old.info)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.nav_msg_type = old.nav_msg_type.value
    msg.prn = old.prn
    msg.clock = type_satnav_clock_to_lcm(old.clock)
    msg.orbit = type_kepler_orbit_to_lcm(old.orbit)
    msg.bgd = old.bgd

    return msg


def lcm_to_metadata_Galileo_ephemeris(
    old: LcmMetadataGalileoEphemeris,
) -> MetadataGalileoEphemeris:
    return MetadataGalileoEphemeris(
        info=lcm_to_type_metadataheader(old.info),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        nav_msg_type=MetadataGalileoEphemerisNavMsgType(old.nav_msg_type),
        prn=old.prn,
        clock=lcm_to_type_satnav_clock(old.clock),
        orbit=lcm_to_type_kepler_orbit(old.orbit),
        bgd=old.bgd,
    )


def metadata_IMU_to_lcm(old: MetadataImu) -> LcmMetadataImu:
    msg = LcmMetadataImu()
    msg.info = type_metadataheader_to_lcm(old.info)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.mounting = type_mounting_to_lcm(old.mounting)
    msg.error_model = old.error_model.value
    msg.error_model_params = old.error_model_params.tolist()
    msg.num_error_model_params = len(old.error_model_params)

    return msg


def lcm_to_metadata_IMU(old: LcmMetadataImu) -> MetadataImu:
    return MetadataImu(
        info=lcm_to_type_metadataheader(old.info),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        mounting=lcm_to_type_mounting(old.mounting),
        error_model=MetadataImuErrorModel(old.error_model),
        error_model_params=np.array(old.error_model_params),
    )


def metadata_generic_to_lcm(old: MetadataGeneric) -> LcmMetadataGeneric:
    msg = LcmMetadataGeneric()
    msg.info = type_metadataheader_to_lcm(old.info)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.mounting = type_mounting_to_lcm(old.mounting)

    return msg


def lcm_to_metadata_generic(old: LcmMetadataGeneric) -> MetadataGeneric:
    return MetadataGeneric(
        info=lcm_to_type_metadataheader(old.info),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        mounting=lcm_to_type_mounting(old.mounting),
    )


def metadata_image_features_to_lcm(
    old: MetadataImageFeatures,
) -> LcmMetadataImageFeatures:
    msg = LcmMetadataImageFeatures()
    msg.info = type_metadataheader_to_lcm(old.info)
    msg.keypoint_detector = old.keypoint_detector.value
    msg.orientation_calculated = old.orientation_calculated
    msg.descriptor_extractor = old.descriptor_extractor.value
    msg.is_bigendian = old.is_bigendian
    msg.descriptor_type = old.descriptor_type.value
    msg.descriptor_number_of_elements = old.descriptor_number_of_elements

    return msg


def lcm_to_metadata_image_features(
    old: LcmMetadataImageFeatures,
) -> MetadataImageFeatures:
    return MetadataImageFeatures(
        info=lcm_to_type_metadataheader(old.info),
        keypoint_detector=MetadataImageFeaturesKeypointDetector(old.keypoint_detector),
        orientation_calculated=old.orientation_calculated,
        descriptor_extractor=MetadataImageFeaturesDescriptorExtractor(
            old.descriptor_extractor
        ),
        is_bigendian=old.is_bigendian,
        descriptor_type=MetadataImageFeaturesDescriptorType(old.descriptor_type),
        descriptor_number_of_elements=old.descriptor_number_of_elements,
    )


def metadata_magnetic_field_to_lcm(
    old: MetadataMagneticField,
) -> LcmMetadataMagneticField:
    msg = LcmMetadataMagneticField()
    msg.info = type_metadataheader_to_lcm(old.info)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.mounting = type_mounting_to_lcm(old.mounting)
    msg.k = old.k.tolist() if old.k is not None else []
    msg.num_meas = len(old.k) if old.k is not None else 0
    msg.b = old.b.tolist() if old.b is not None else []

    return msg


def lcm_to_metadata_magnetic_field(
    old: LcmMetadataMagneticField,
) -> MetadataMagneticField:
    return MetadataMagneticField(
        info=lcm_to_type_metadataheader(old.info),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        mounting=lcm_to_type_mounting(old.mounting),
        k=np.array(old.k),
        b=np.array(old.b),
    )


def metadata_satnav_obs_to_lcm(old: MetadataSatnavObs) -> LcmMetadataSatnavObs:
    msg = LcmMetadataSatnavObs()
    msg.info = type_metadataheader_to_lcm(old.info)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.deltarange_interval_start = (
        old.deltarange_interval_start
        if old.deltarange_interval_start is not None
        else float()
    )
    msg.deltarange_interval_stop = (
        old.deltarange_interval_stop
        if old.deltarange_interval_stop is not None
        else float()
    )

    return msg


def lcm_to_metadata_satnav_obs(old: LcmMetadataSatnavObs) -> MetadataSatnavObs:
    return MetadataSatnavObs(
        info=lcm_to_type_metadataheader(old.info),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        deltarange_interval_start=old.deltarange_interval_start,
        deltarange_interval_stop=old.deltarange_interval_stop,
    )


def measurement_IMU_to_lcm(old: MeasurementImu) -> LcmMeasurementImu:
    msg = LcmMeasurementImu()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.imu_type = old.imu_type.value
    msg.meas_accel = old.meas_accel.tolist()
    msg.meas_gyro = old.meas_gyro.tolist()
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_IMU(old: LcmMeasurementImu) -> MeasurementImu:
    return MeasurementImu(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        imu_type=MeasurementImuImuType(old.imu_type),
        meas_accel=np.array(old.meas_accel),
        meas_gyro=np.array(old.meas_gyro),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_TDOA_1Tx_2Rx_to_lcm(
    old: MeasurementTdoa1Tx2Rx,
) -> LcmMeasurementTdoa1Tx2Rx:
    msg = LcmMeasurementTdoa1Tx2Rx()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.tx_position_and_covariance = type_remote_point_to_lcm(
        old.tx_position_and_covariance
    )
    msg.rx1_position_and_covariance = type_remote_point_to_lcm(
        old.rx1_position_and_covariance
    )
    msg.obs = old.obs
    msg.variance = old.variance
    msg.error_model = old.error_model.value
    msg.error_model_params = old.error_model_params.tolist()
    msg.num_error_model_params = len(old.error_model_params)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_TDOA_1Tx_2Rx(
    old: LcmMeasurementTdoa1Tx2Rx,
) -> MeasurementTdoa1Tx2Rx:
    return MeasurementTdoa1Tx2Rx(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        tx_position_and_covariance=lcm_to_type_remote_point(
            old.tx_position_and_covariance
        ),
        rx1_position_and_covariance=lcm_to_type_remote_point(
            old.rx1_position_and_covariance
        ),
        obs=old.obs,
        variance=old.variance,
        error_model=MeasurementTdoa1Tx2RxErrorModel(old.error_model),
        error_model_params=np.array(old.error_model_params),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_TDOA_2Tx_1Rx_to_lcm(
    old: MeasurementTdoa2Tx1Rx,
) -> LcmMeasurementTdoa2Tx1Rx:
    msg = LcmMeasurementTdoa2Tx1Rx()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.tx1_position_and_covariance = type_remote_point_to_lcm(
        old.tx1_position_and_covariance
    )
    msg.tx2_position_and_covariance = type_remote_point_to_lcm(
        old.tx2_position_and_covariance
    )
    msg.obs = old.obs
    msg.variance = old.variance
    msg.error_model = old.error_model.value
    msg.error_model_params = old.error_model_params.tolist()
    msg.num_error_model_params = len(old.error_model_params)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_TDOA_2Tx_1Rx(
    old: LcmMeasurementTdoa2Tx1Rx,
) -> MeasurementTdoa2Tx1Rx:
    return MeasurementTdoa2Tx1Rx(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        tx1_position_and_covariance=lcm_to_type_remote_point(
            old.tx1_position_and_covariance
        ),
        tx2_position_and_covariance=lcm_to_type_remote_point(
            old.tx2_position_and_covariance
        ),
        obs=old.obs,
        variance=old.variance,
        error_model=MeasurementTdoa2Tx1RxErrorModel(old.error_model),
        error_model_params=np.array(old.error_model_params),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_accumulated_distance_traveled_to_lcm(
    old: MeasurementAccumulatedDistanceTraveled,
) -> LcmMeasurementAccumulatedDistanceTraveled:
    msg = LcmMeasurementAccumulatedDistanceTraveled()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.delta_t = old.delta_t
    msg.obs = old.obs
    msg.variance = old.variance
    msg.error_model = old.error_model.value
    msg.error_model_params = old.error_model_params.tolist()
    msg.num_error_model_params = len(old.error_model_params)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_accumulated_distance_traveled(
    old: LcmMeasurementAccumulatedDistanceTraveled,
) -> MeasurementAccumulatedDistanceTraveled:
    return MeasurementAccumulatedDistanceTraveled(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        delta_t=old.delta_t,
        obs=old.obs,
        variance=old.variance,
        error_model=MeasurementAccumulatedDistanceTraveledErrorModel(old.error_model),
        error_model_params=np.array(old.error_model_params),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_altitude_to_lcm(old: MeasurementAltitude) -> LcmMeasurementAltitude:
    msg = LcmMeasurementAltitude()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.reference = old.reference.value
    msg.altitude = old.altitude
    msg.variance = old.variance
    msg.error_model = old.error_model.value
    msg.error_model_params = old.error_model_params.tolist()
    msg.num_error_model_params = len(old.error_model_params)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_altitude(old: LcmMeasurementAltitude) -> MeasurementAltitude:
    return MeasurementAltitude(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        reference=MeasurementAltitudeReference(old.reference),
        altitude=old.altitude,
        variance=old.variance,
        error_model=MeasurementAltitudeErrorModel(old.error_model),
        error_model_params=np.array(old.error_model_params),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_angular_velocity_to_lcm(
    old: MeasurementAngularVelocity,
) -> LcmMeasurementAngularVelocity:
    msg = LcmMeasurementAngularVelocity()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.reference_frame = old.reference_frame.value
    msg.imu_type = old.imu_type.value
    msg.meas = old.meas.tolist()
    msg.covariance = old.covariance.tolist()
    msg.error_model = old.error_model.value
    msg.error_model_params = old.error_model_params.tolist()
    msg.num_error_model_params = len(old.error_model_params)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_angular_velocity(
    old: LcmMeasurementAngularVelocity,
) -> MeasurementAngularVelocity:
    return MeasurementAngularVelocity(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        reference_frame=MeasurementAngularVelocityReferenceFrame(old.reference_frame),
        imu_type=MeasurementAngularVelocityImuType(old.imu_type),
        meas=np.array(old.meas),
        covariance=np.array(old.covariance),
        error_model=MeasurementAngularVelocityErrorModel(old.error_model),
        error_model_params=np.array(old.error_model_params),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_angular_velocity_1d_to_lcm(
    old: MeasurementAngularVelocity1D,
) -> LcmMeasurementAngularVelocity1D:
    msg = LcmMeasurementAngularVelocity1D()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.sensor_type = old.sensor_type.value
    msg.obs = old.obs
    msg.variance = old.variance
    msg.error_model = old.error_model.value
    msg.error_model_params = old.error_model_params.tolist()
    msg.num_error_model_params = len(old.error_model_params)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_angular_velocity_1d(
    old: LcmMeasurementAngularVelocity1D,
) -> MeasurementAngularVelocity1D:
    return MeasurementAngularVelocity1D(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        sensor_type=MeasurementAngularVelocity1DSensorType(old.sensor_type),
        obs=old.obs,
        variance=old.variance,
        error_model=MeasurementAngularVelocity1DErrorModel(old.error_model),
        error_model_params=np.array(old.error_model_params),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_attitude_2d_to_lcm(
    old: MeasurementAttitude2D,
) -> LcmMeasurementAttitude2D:
    msg = LcmMeasurementAttitude2D()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.reference_frame = old.reference_frame.value
    msg.attitude2d = old.attitude2d.tolist()
    msg.covariance = old.covariance.tolist()
    msg.error_model = old.error_model.value
    msg.error_model_params = old.error_model_params.tolist()
    msg.num_error_model_params = len(old.error_model_params)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_attitude_2d(
    old: LcmMeasurementAttitude2D,
) -> MeasurementAttitude2D:
    return MeasurementAttitude2D(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        reference_frame=MeasurementAttitude2DReferenceFrame(old.reference_frame),
        attitude2d=np.array(old.attitude2d),
        covariance=np.array(old.covariance),
        error_model=MeasurementAttitude2DErrorModel(old.error_model),
        error_model_params=np.array(old.error_model_params),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_attitude_3d_to_lcm(
    old: MeasurementAttitude3D,
) -> LcmMeasurementAttitude3D:
    msg = LcmMeasurementAttitude3D()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.reference_frame = old.reference_frame.value
    msg.quaternion = old.quaternion.tolist()
    msg.tilt_error_covariance = old.tilt_error_covariance.tolist()
    msg.error_model = old.error_model.value
    msg.error_model_params = old.error_model_params.tolist()
    msg.num_error_model_params = len(old.error_model_params)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_attitude_3d(
    old: LcmMeasurementAttitude3D,
) -> MeasurementAttitude3D:
    return MeasurementAttitude3D(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        reference_frame=MeasurementAttitude3DReferenceFrame(old.reference_frame),
        quaternion=np.array(old.quaternion),
        tilt_error_covariance=np.array(old.tilt_error_covariance),
        error_model=MeasurementAttitude3DErrorModel(old.error_model),
        error_model_params=np.array(old.error_model_params),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_barometer_to_lcm(old: MeasurementBarometer) -> LcmMeasurementBarometer:
    msg = LcmMeasurementBarometer()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.pressure = old.pressure
    msg.variance = old.variance
    msg.error_model = old.error_model.value
    msg.error_model_params = old.error_model_params.tolist()
    msg.num_error_model_params = len(old.error_model_params)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_barometer(old: LcmMeasurementBarometer) -> MeasurementBarometer:
    return MeasurementBarometer(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        pressure=old.pressure,
        variance=old.variance,
        error_model=MeasurementBarometerErrorModel(old.error_model),
        error_model_params=np.array(old.error_model_params),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_delta_position_to_lcm(
    old: MeasurementDeltaPosition,
) -> LcmMeasurementDeltaPosition:
    msg = LcmMeasurementDeltaPosition()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.reference_frame = old.reference_frame.value
    msg.delta_t = old.delta_t
    msg.term1 = old.term1 if old.term1 is not None else float()
    msg.term2 = old.term2 if old.term2 is not None else float()
    msg.term3 = old.term3 if old.term3 is not None else float()
    msg.covariance = old.covariance.tolist()
    msg.num_meas = len(old.covariance)
    msg.error_model = old.error_model.value
    msg.error_model_params = old.error_model_params.tolist()
    msg.num_error_model_params = len(old.error_model_params)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_delta_position(
    old: LcmMeasurementDeltaPosition,
) -> MeasurementDeltaPosition:
    return MeasurementDeltaPosition(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        reference_frame=MeasurementDeltaPositionReferenceFrame(old.reference_frame),
        delta_t=old.delta_t,
        term1=old.term1,
        term2=old.term2,
        term3=old.term3,
        covariance=np.array(old.covariance),
        error_model=MeasurementDeltaPositionErrorModel(old.error_model),
        error_model_params=np.array(old.error_model_params),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_delta_range_to_lcm(
    old: MeasurementDeltaRange,
) -> LcmMeasurementDeltaRange:
    msg = LcmMeasurementDeltaRange()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.delta_t = old.delta_t
    msg.obs = old.obs
    msg.variance = old.variance
    msg.error_model = old.error_model.value
    msg.error_model_params = old.error_model_params.tolist()
    msg.num_error_model_params = len(old.error_model_params)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_delta_range(
    old: LcmMeasurementDeltaRange,
) -> MeasurementDeltaRange:
    return MeasurementDeltaRange(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        delta_t=old.delta_t,
        obs=old.obs,
        variance=old.variance,
        error_model=MeasurementDeltaRangeErrorModel(old.error_model),
        error_model_params=np.array(old.error_model_params),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_delta_range_to_point_to_lcm(
    old: MeasurementDeltaRangeToPoint,
) -> LcmMeasurementDeltaRangeToPoint:
    msg = LcmMeasurementDeltaRangeToPoint()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.remote_point = type_remote_point_to_lcm(old.remote_point)
    msg.obs = old.obs
    msg.delta_t = old.delta_t
    msg.variance = old.variance
    msg.error_model = old.error_model.value
    msg.error_model_params = old.error_model_params.tolist()
    msg.num_error_model_params = len(old.error_model_params)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_delta_range_to_point(
    old: LcmMeasurementDeltaRangeToPoint,
) -> MeasurementDeltaRangeToPoint:
    return MeasurementDeltaRangeToPoint(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        remote_point=lcm_to_type_remote_point(old.remote_point),
        obs=old.obs,
        delta_t=old.delta_t,
        variance=old.variance,
        error_model=MeasurementDeltaRangeToPointErrorModel(old.error_model),
        error_model_params=np.array(old.error_model_params),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_direction_2d_to_points_to_lcm(
    old: MeasurementDirection2DToPoints,
) -> LcmMeasurementDirection2DToPoints:
    msg = LcmMeasurementDirection2DToPoints()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.obs = [type_direction_2d_to_point_to_lcm(x) for x in old.obs]
    msg.num_obs = len(old.obs)

    return msg


def lcm_to_measurement_direction_2d_to_points(
    old: LcmMeasurementDirection2DToPoints,
) -> MeasurementDirection2DToPoints:
    return MeasurementDirection2DToPoints(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        obs=[lcm_to_type_direction_2d_to_point(x) for x in old.obs],
    )


def measurement_direction_3d_to_points_to_lcm(
    old: MeasurementDirection3DToPoints,
) -> LcmMeasurementDirection3DToPoints:
    msg = LcmMeasurementDirection3DToPoints()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.obs = [type_direction_3d_to_point_to_lcm(x) for x in old.obs]
    msg.num_obs = len(old.obs)

    return msg


def lcm_to_measurement_direction_3d_to_points(
    old: LcmMeasurementDirection3DToPoints,
) -> MeasurementDirection3DToPoints:
    return MeasurementDirection3DToPoints(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        obs=[lcm_to_type_direction_3d_to_point(x) for x in old.obs],
    )


def measurement_direction_of_motion_2d_to_lcm(
    old: MeasurementDirectionOfMotion2D,
) -> LcmMeasurementDirectionOfMotion2D:
    msg = LcmMeasurementDirectionOfMotion2D()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.reference = old.reference.value
    msg.obs = old.obs
    msg.variance = old.variance
    msg.error_model = old.error_model.value
    msg.error_model_params = old.error_model_params.tolist()
    msg.num_error_model_params = len(old.error_model_params)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_direction_of_motion_2d(
    old: LcmMeasurementDirectionOfMotion2D,
) -> MeasurementDirectionOfMotion2D:
    return MeasurementDirectionOfMotion2D(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        reference=MeasurementDirectionOfMotion2DReference(old.reference),
        obs=old.obs,
        variance=old.variance,
        error_model=MeasurementDirectionOfMotion2DErrorModel(old.error_model),
        error_model_params=np.array(old.error_model_params),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_direction_of_motion_3d_to_lcm(
    old: MeasurementDirectionOfMotion3D,
) -> LcmMeasurementDirectionOfMotion3D:
    msg = LcmMeasurementDirectionOfMotion3D()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.reference_frame = old.reference_frame.value
    msg.obs = old.obs.tolist()
    msg.error_vector = old.error_vector.tolist()
    msg.covariance = old.covariance.tolist()
    msg.error_model = old.error_model.value
    msg.error_model_params = old.error_model_params.tolist()
    msg.num_error_model_params = len(old.error_model_params)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_direction_of_motion_3d(
    old: LcmMeasurementDirectionOfMotion3D,
) -> MeasurementDirectionOfMotion3D:
    return MeasurementDirectionOfMotion3D(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        reference_frame=MeasurementDirectionOfMotion3DReferenceFrame(
            old.reference_frame
        ),
        obs=np.array(old.obs),
        error_vector=np.array(old.error_vector),
        covariance=np.array(old.covariance),
        error_model=MeasurementDirectionOfMotion3DErrorModel(old.error_model),
        error_model_params=np.array(old.error_model_params),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_frequency_difference_to_lcm(
    old: MeasurementFrequencyDifference,
) -> LcmMeasurementFrequencyDifference:
    msg = LcmMeasurementFrequencyDifference()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.time_of_validity_attosec = old.time_of_validity_attosec
    msg.clock_id1 = old.clock_id1
    msg.clock_id2 = old.clock_id2
    msg.freq_diff = old.freq_diff
    msg.variance = old.variance
    msg.error_model = old.error_model.value
    msg.error_model_params = old.error_model_params.tolist()
    msg.num_error_model_params = len(old.error_model_params)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_frequency_difference(
    old: LcmMeasurementFrequencyDifference,
) -> MeasurementFrequencyDifference:
    return MeasurementFrequencyDifference(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        time_of_validity_attosec=old.time_of_validity_attosec,
        clock_id1=old.clock_id1,
        clock_id2=old.clock_id2,
        freq_diff=old.freq_diff,
        variance=old.variance,
        error_model=MeasurementFrequencyDifferenceErrorModel(old.error_model),
        error_model_params=np.array(old.error_model_params),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_heading_to_lcm(old: MeasurementHeading) -> LcmMeasurementHeading:
    msg = LcmMeasurementHeading()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.reference = old.reference.value
    msg.obs = old.obs
    msg.variance = old.variance
    msg.error_model = old.error_model.value
    msg.error_model_params = old.error_model_params.tolist()
    msg.num_error_model_params = len(old.error_model_params)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_heading(old: LcmMeasurementHeading) -> MeasurementHeading:
    return MeasurementHeading(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        reference=MeasurementHeadingReference(old.reference),
        obs=old.obs,
        variance=old.variance,
        error_model=MeasurementHeadingErrorModel(old.error_model),
        error_model_params=np.array(old.error_model_params),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def image_to_lcm(old: Image) -> LcmImage:
    msg = LcmImage()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.height = old.height
    msg.width = old.width
    msg.is_bigendian = old.is_bigendian
    msg.image_type = old.image_type.value
    msg.image_data = old.image_data.tolist()
    msg.image_data_length = len(old.image_data)
    msg.camera_model = old.camera_model.value
    msg.model_coefficients = old.model_coefficients.tolist()
    msg.num_model_coefficients = len(old.model_coefficients)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_image(old: LcmImage) -> Image:
    return Image(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        height=old.height,
        width=old.width,
        is_bigendian=old.is_bigendian,
        image_type=ImageImageType(old.image_type),
        image_data=np.array(old.image_data),
        camera_model=ImageCameraModel(old.camera_model),
        model_coefficients=np.array(old.model_coefficients),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_magnetic_field_to_lcm(
    old: MeasurementMagneticField,
) -> LcmMeasurementMagneticField:
    msg = LcmMeasurementMagneticField()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.x_field_strength = (
        old.x_field_strength if old.x_field_strength is not None else float()
    )
    msg.y_field_strength = (
        old.y_field_strength if old.y_field_strength is not None else float()
    )
    msg.z_field_strength = (
        old.z_field_strength if old.z_field_strength is not None else float()
    )
    msg.covariance = old.covariance.tolist()
    msg.num_meas = len(old.covariance)
    msg.error_model = old.error_model.value
    msg.error_model_params = old.error_model_params.tolist()
    msg.num_error_model_params = len(old.error_model_params)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_magnetic_field(
    old: LcmMeasurementMagneticField,
) -> MeasurementMagneticField:
    return MeasurementMagneticField(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        x_field_strength=old.x_field_strength,
        y_field_strength=old.y_field_strength,
        z_field_strength=old.z_field_strength,
        covariance=np.array(old.covariance),
        error_model=MeasurementMagneticFieldErrorModel(old.error_model),
        error_model_params=np.array(old.error_model_params),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_magnetic_field_magnitude_to_lcm(
    old: MeasurementMagneticFieldMagnitude,
) -> LcmMeasurementMagneticFieldMagnitude:
    msg = LcmMeasurementMagneticFieldMagnitude()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.field_strength = old.field_strength
    msg.variance = old.variance
    msg.error_model = old.error_model.value
    msg.error_model_params = old.error_model_params.tolist()
    msg.num_error_model_params = len(old.error_model_params)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_magnetic_field_magnitude(
    old: LcmMeasurementMagneticFieldMagnitude,
) -> MeasurementMagneticFieldMagnitude:
    return MeasurementMagneticFieldMagnitude(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        field_strength=old.field_strength,
        variance=old.variance,
        error_model=MeasurementMagneticFieldMagnitudeErrorModel(old.error_model),
        error_model_params=np.array(old.error_model_params),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_navsim_satnav_with_sv_data_to_lcm(
    old: MeasurementNavsimSatnavWithSvData,
) -> LcmMeasurementNavsimSatnavWithSvData:
    msg = LcmMeasurementNavsimSatnavWithSvData()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.receiver_clock_time = type_satnav_time_to_lcm(old.receiver_clock_time)
    msg.num_signal_types = old.num_signal_types
    msg.obs = [type_navsim_satnav_obs_to_lcm(x) for x in old.obs]
    msg.num_signals_tracked = len(old.obs)
    msg.sv_data = [type_navsim_satnav_sv_data_to_lcm(x) for x in old.sv_data]
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_navsim_satnav_with_sv_data(
    old: LcmMeasurementNavsimSatnavWithSvData,
) -> MeasurementNavsimSatnavWithSvData:
    return MeasurementNavsimSatnavWithSvData(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        receiver_clock_time=lcm_to_type_satnav_time(old.receiver_clock_time),
        num_signal_types=old.num_signal_types,
        obs=[lcm_to_type_navsim_satnav_obs(x) for x in old.obs],
        sv_data=[lcm_to_type_navsim_satnav_sv_data(x) for x in old.sv_data],
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_position_to_lcm(old: MeasurementPosition) -> LcmMeasurementPosition:
    msg = LcmMeasurementPosition()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.reference_frame = old.reference_frame.value
    msg.term1 = old.term1 if old.term1 is not None else float()
    msg.term2 = old.term2 if old.term2 is not None else float()
    msg.term3 = old.term3 if old.term3 is not None else float()
    msg.covariance = old.covariance.tolist()
    msg.num_meas = len(old.covariance)
    msg.error_model = old.error_model.value
    msg.error_model_params = old.error_model_params.tolist()
    msg.num_error_model_params = len(old.error_model_params)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_position(old: LcmMeasurementPosition) -> MeasurementPosition:
    return MeasurementPosition(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        reference_frame=MeasurementPositionReferenceFrame(old.reference_frame),
        term1=old.term1,
        term2=old.term2,
        term3=old.term3,
        covariance=np.array(old.covariance),
        error_model=MeasurementPositionErrorModel(old.error_model),
        error_model_params=np.array(old.error_model_params),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_position_attitude_to_lcm(
    old: MeasurementPositionAttitude,
) -> LcmMeasurementPositionAttitude:
    msg = LcmMeasurementPositionAttitude()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.reference_frame = old.reference_frame.value
    msg.p1 = old.p1
    msg.p2 = old.p2
    msg.p3 = old.p3
    msg.quaternion = old.quaternion.tolist()
    msg.covariance = old.covariance.tolist()
    msg.error_model = old.error_model.value
    msg.error_model_params = old.error_model_params.tolist()
    msg.num_error_model_params = len(old.error_model_params)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_position_attitude(
    old: LcmMeasurementPositionAttitude,
) -> MeasurementPositionAttitude:
    return MeasurementPositionAttitude(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        reference_frame=MeasurementPositionAttitudeReferenceFrame(old.reference_frame),
        p1=old.p1,
        p2=old.p2,
        p3=old.p3,
        quaternion=np.array(old.quaternion),
        covariance=np.array(old.covariance),
        error_model=MeasurementPositionAttitudeErrorModel(old.error_model),
        error_model_params=np.array(old.error_model_params),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_position_velocity_attitude_to_lcm(
    old: MeasurementPositionVelocityAttitude,
) -> LcmMeasurementPositionVelocityAttitude:
    msg = LcmMeasurementPositionVelocityAttitude()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.reference_frame = old.reference_frame.value
    msg.p1 = old.p1 if old.p1 is not None else float()
    msg.p2 = old.p2 if old.p2 is not None else float()
    msg.p3 = old.p3 if old.p3 is not None else float()
    msg.v1 = old.v1 if old.v1 is not None else float()
    msg.v2 = old.v2 if old.v2 is not None else float()
    msg.v3 = old.v3 if old.v3 is not None else float()
    msg.quaternion = old.quaternion.tolist() if old.quaternion is not None else []
    msg.covariance = old.covariance.tolist()
    msg.num_meas = len(old.covariance)
    msg.error_model = old.error_model.value
    msg.error_model_params = old.error_model_params.tolist()
    msg.num_error_model_params = len(old.error_model_params)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_position_velocity_attitude(
    old: LcmMeasurementPositionVelocityAttitude,
) -> MeasurementPositionVelocityAttitude:
    return MeasurementPositionVelocityAttitude(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        reference_frame=MeasurementPositionVelocityAttitudeReferenceFrame(
            old.reference_frame
        ),
        p1=old.p1,
        p2=old.p2,
        p3=old.p3,
        v1=old.v1,
        v2=old.v2,
        v3=old.v3,
        quaternion=np.array(old.quaternion),
        covariance=np.array(old.covariance),
        error_model=MeasurementPositionVelocityAttitudeErrorModel(old.error_model),
        error_model_params=np.array(old.error_model_params),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_range_rate_to_point_to_lcm(
    old: MeasurementRangeRateToPoint,
) -> LcmMeasurementRangeRateToPoint:
    msg = LcmMeasurementRangeRateToPoint()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.remote_point = type_remote_point_to_lcm(old.remote_point)
    msg.obs = old.obs
    msg.variance = old.variance
    msg.error_model = old.error_model.value
    msg.error_model_params = old.error_model_params.tolist()
    msg.num_error_model_params = len(old.error_model_params)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_range_rate_to_point(
    old: LcmMeasurementRangeRateToPoint,
) -> MeasurementRangeRateToPoint:
    return MeasurementRangeRateToPoint(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        remote_point=lcm_to_type_remote_point(old.remote_point),
        obs=old.obs,
        variance=old.variance,
        error_model=MeasurementRangeRateToPointErrorModel(old.error_model),
        error_model_params=np.array(old.error_model_params),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_range_to_point_to_lcm(
    old: MeasurementRangeToPoint,
) -> LcmMeasurementRangeToPoint:
    msg = LcmMeasurementRangeToPoint()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.remote_point = type_remote_point_to_lcm(old.remote_point)
    msg.obs = old.obs
    msg.variance = old.variance
    msg.error_model = old.error_model.value
    msg.error_model_params = old.error_model_params.tolist()
    msg.num_error_model_params = len(old.error_model_params)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_range_to_point(
    old: LcmMeasurementRangeToPoint,
) -> MeasurementRangeToPoint:
    return MeasurementRangeToPoint(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        remote_point=lcm_to_type_remote_point(old.remote_point),
        obs=old.obs,
        variance=old.variance,
        error_model=MeasurementRangeToPointErrorModel(old.error_model),
        error_model_params=np.array(old.error_model_params),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_satnav_to_lcm(old: MeasurementSatnav) -> LcmMeasurementSatnav:
    msg = LcmMeasurementSatnav()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.receiver_clock_time = type_satnav_time_to_lcm(old.receiver_clock_time)
    msg.num_signal_types = old.num_signal_types
    msg.obs = [type_satnav_obs_to_lcm(x) for x in old.obs]
    msg.num_signals_tracked = len(old.obs)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_satnav(old: LcmMeasurementSatnav) -> MeasurementSatnav:
    return MeasurementSatnav(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        receiver_clock_time=lcm_to_type_satnav_time(old.receiver_clock_time),
        num_signal_types=old.num_signal_types,
        obs=[lcm_to_type_satnav_obs(x) for x in old.obs],
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_satnav_subframe_to_lcm(
    old: MeasurementSatnavSubframe,
) -> LcmMeasurementSatnavSubframe:
    msg = LcmMeasurementSatnavSubframe()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.gnss_message_id = old.gnss_message_id
    msg.prn = old.prn
    msg.satellite_system = type_satnav_satellite_system_to_lcm(old.satellite_system)
    msg.freq_slot_id = old.freq_slot_id
    msg.data_vector = old.data_vector.tolist()
    msg.num_bytes = len(old.data_vector)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_satnav_subframe(
    old: LcmMeasurementSatnavSubframe,
) -> MeasurementSatnavSubframe:
    return MeasurementSatnavSubframe(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        gnss_message_id=old.gnss_message_id,
        prn=old.prn,
        satellite_system=lcm_to_type_satnav_satellite_system(old.satellite_system),
        freq_slot_id=old.freq_slot_id,
        data_vector=np.array(old.data_vector),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_satnav_with_sv_data_to_lcm(
    old: MeasurementSatnavWithSvData,
) -> LcmMeasurementSatnavWithSvData:
    msg = LcmMeasurementSatnavWithSvData()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.receiver_clock_time = type_satnav_time_to_lcm(old.receiver_clock_time)
    msg.num_signal_types = old.num_signal_types
    msg.obs = [type_satnav_obs_to_lcm(x) for x in old.obs]
    msg.num_signals_tracked = len(old.obs)
    msg.sv_data = [type_satnav_sv_data_to_lcm(x) for x in old.sv_data]
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_satnav_with_sv_data(
    old: LcmMeasurementSatnavWithSvData,
) -> MeasurementSatnavWithSvData:
    return MeasurementSatnavWithSvData(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        receiver_clock_time=lcm_to_type_satnav_time(old.receiver_clock_time),
        num_signal_types=old.num_signal_types,
        obs=[lcm_to_type_satnav_obs(x) for x in old.obs],
        sv_data=[lcm_to_type_satnav_sv_data(x) for x in old.sv_data],
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_specific_force_1d_to_lcm(
    old: MeasurementSpecificForce1D,
) -> LcmMeasurementSpecificForce1D:
    msg = LcmMeasurementSpecificForce1D()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.sensor_type = old.sensor_type.value
    msg.obs = old.obs
    msg.variance = old.variance
    msg.error_model = old.error_model.value
    msg.error_model_params = old.error_model_params.tolist()
    msg.num_error_model_params = len(old.error_model_params)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_specific_force_1d(
    old: LcmMeasurementSpecificForce1D,
) -> MeasurementSpecificForce1D:
    return MeasurementSpecificForce1D(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        sensor_type=MeasurementSpecificForce1DSensorType(old.sensor_type),
        obs=old.obs,
        variance=old.variance,
        error_model=MeasurementSpecificForce1DErrorModel(old.error_model),
        error_model_params=np.array(old.error_model_params),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_speed_to_lcm(old: MeasurementSpeed) -> LcmMeasurementSpeed:
    msg = LcmMeasurementSpeed()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.reference = old.reference.value
    msg.speed = old.speed
    msg.variance = old.variance
    msg.error_model = old.error_model.value
    msg.error_model_params = old.error_model_params.tolist()
    msg.num_error_model_params = len(old.error_model_params)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_speed(old: LcmMeasurementSpeed) -> MeasurementSpeed:
    return MeasurementSpeed(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        reference=MeasurementSpeedReference(old.reference),
        speed=old.speed,
        variance=old.variance,
        error_model=MeasurementSpeedErrorModel(old.error_model),
        error_model_params=np.array(old.error_model_params),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_temperature_to_lcm(
    old: MeasurementTemperature,
) -> LcmMeasurementTemperature:
    msg = LcmMeasurementTemperature()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.temperature = old.temperature
    msg.variance = old.variance
    msg.error_model = old.error_model.value
    msg.error_model_params = old.error_model_params.tolist()
    msg.num_error_model_params = len(old.error_model_params)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_temperature(
    old: LcmMeasurementTemperature,
) -> MeasurementTemperature:
    return MeasurementTemperature(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        temperature=old.temperature,
        variance=old.variance,
        error_model=MeasurementTemperatureErrorModel(old.error_model),
        error_model_params=np.array(old.error_model_params),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_time_to_lcm(old: MeasurementTime) -> LcmMeasurementTime:
    msg = LcmMeasurementTime()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.time_of_validity_attosec = old.time_of_validity_attosec
    msg.clock_id = old.clock_id.tolist()
    msg.num_obs = len(old.clock_id)
    msg.elapsed_nsec = old.elapsed_nsec.tolist()
    msg.elapsed_attosec = old.elapsed_attosec.tolist()
    msg.digits_of_precision = old.digits_of_precision
    msg.covariance = old.covariance.tolist()
    msg.error_model = old.error_model.value
    msg.error_model_params = old.error_model_params.tolist()
    msg.num_error_model_params = len(old.error_model_params)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_time(old: LcmMeasurementTime) -> MeasurementTime:
    return MeasurementTime(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        time_of_validity_attosec=old.time_of_validity_attosec,
        clock_id=np.array(old.clock_id),
        elapsed_nsec=np.array(old.elapsed_nsec),
        elapsed_attosec=np.array(old.elapsed_attosec),
        digits_of_precision=old.digits_of_precision,
        covariance=np.array(old.covariance),
        error_model=MeasurementTimeErrorModel(old.error_model),
        error_model_params=np.array(old.error_model_params),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_time_difference_to_lcm(
    old: MeasurementTimeDifference,
) -> LcmMeasurementTimeDifference:
    msg = LcmMeasurementTimeDifference()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.time_of_validity_attosec = old.time_of_validity_attosec
    msg.clock_id1 = old.clock_id1
    msg.clock_id2 = old.clock_id2
    msg.time_diff_nsec = old.time_diff_nsec
    msg.time_diff_attosec = old.time_diff_attosec
    msg.digits_of_precision = old.digits_of_precision
    msg.variance = old.variance
    msg.error_model = old.error_model.value
    msg.error_model_params = old.error_model_params.tolist()
    msg.num_error_model_params = len(old.error_model_params)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_time_difference(
    old: LcmMeasurementTimeDifference,
) -> MeasurementTimeDifference:
    return MeasurementTimeDifference(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        time_of_validity_attosec=old.time_of_validity_attosec,
        clock_id1=old.clock_id1,
        clock_id2=old.clock_id2,
        time_diff_nsec=old.time_diff_nsec,
        time_diff_attosec=old.time_diff_attosec,
        digits_of_precision=old.digits_of_precision,
        variance=old.variance,
        error_model=MeasurementTimeDifferenceErrorModel(old.error_model),
        error_model_params=np.array(old.error_model_params),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_time_frequency_difference_to_lcm(
    old: MeasurementTimeFrequencyDifference,
) -> LcmMeasurementTimeFrequencyDifference:
    msg = LcmMeasurementTimeFrequencyDifference()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.time_of_validity_attosec = old.time_of_validity_attosec
    msg.clock_id1 = old.clock_id1
    msg.clock_id2 = old.clock_id2
    msg.time_diff_nsec = old.time_diff_nsec
    msg.time_diff_attosec = old.time_diff_attosec
    msg.digits_of_precision = old.digits_of_precision
    msg.freq_diff = old.freq_diff
    msg.covariance = old.covariance.tolist()
    msg.error_model = old.error_model.value
    msg.error_model_params = old.error_model_params.tolist()
    msg.num_error_model_params = len(old.error_model_params)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_time_frequency_difference(
    old: LcmMeasurementTimeFrequencyDifference,
) -> MeasurementTimeFrequencyDifference:
    return MeasurementTimeFrequencyDifference(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        time_of_validity_attosec=old.time_of_validity_attosec,
        clock_id1=old.clock_id1,
        clock_id2=old.clock_id2,
        time_diff_nsec=old.time_diff_nsec,
        time_diff_attosec=old.time_diff_attosec,
        digits_of_precision=old.digits_of_precision,
        freq_diff=old.freq_diff,
        covariance=np.array(old.covariance),
        error_model=MeasurementTimeFrequencyDifferenceErrorModel(old.error_model),
        error_model_params=np.array(old.error_model_params),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


def measurement_velocity_to_lcm(old: MeasurementVelocity) -> LcmMeasurementVelocity:
    msg = LcmMeasurementVelocity()
    msg.header = type_header_to_lcm(old.header)
    msg.time_of_validity = type_timestamp_to_lcm(old.time_of_validity)
    msg.reference_frame = old.reference_frame.value
    msg.x = old.x if old.x is not None else float()
    msg.y = old.y if old.y is not None else float()
    msg.z = old.z if old.z is not None else float()
    msg.covariance = old.covariance.tolist()
    msg.num_meas = len(old.covariance)
    msg.error_model = old.error_model.value
    msg.error_model_params = old.error_model_params.tolist()
    msg.num_error_model_params = len(old.error_model_params)
    msg.integrity = [type_integrity_to_lcm(x) for x in old.integrity]
    msg.num_integrity = len(old.integrity)

    return msg


def lcm_to_measurement_velocity(old: LcmMeasurementVelocity) -> MeasurementVelocity:
    return MeasurementVelocity(
        header=lcm_to_type_header(old.header),
        time_of_validity=lcm_to_type_timestamp(old.time_of_validity),
        reference_frame=MeasurementVelocityReferenceFrame(old.reference_frame),
        x=old.x,
        y=old.y,
        z=old.z,
        covariance=np.array(old.covariance),
        error_model=MeasurementVelocityErrorModel(old.error_model),
        error_model_params=np.array(old.error_model_params),
        integrity=[lcm_to_type_integrity(x) for x in old.integrity],
    )


AspnMsg: TypeAlias = Union[
    MetadataBeidouEphemeris,
    MetadataGlonassEphemeris,
    MetadataGpsCnavEphemeris,
    MetadataGpsLnavEphemeris,
    MetadataGpsMnavEphemeris,
    MetadataGpsIonoUtcParameters,
    MetadataGalileoEphemeris,
    MetadataImu,
    MetadataGeneric,
    MetadataImageFeatures,
    MetadataMagneticField,
    MetadataSatnavObs,
    MeasurementImu,
    MeasurementTdoa1Tx2Rx,
    MeasurementTdoa2Tx1Rx,
    MeasurementAccumulatedDistanceTraveled,
    MeasurementAltitude,
    MeasurementAngularVelocity,
    MeasurementAngularVelocity1D,
    MeasurementAttitude2D,
    MeasurementAttitude3D,
    MeasurementBarometer,
    MeasurementDeltaPosition,
    MeasurementDeltaRange,
    MeasurementDeltaRangeToPoint,
    MeasurementDirection2DToPoints,
    MeasurementDirection3DToPoints,
    MeasurementDirectionOfMotion2D,
    MeasurementDirectionOfMotion3D,
    MeasurementFrequencyDifference,
    MeasurementHeading,
    Image,
    MeasurementMagneticField,
    MeasurementMagneticFieldMagnitude,
    MeasurementNavsimSatnavWithSvData,
    MeasurementPosition,
    MeasurementPositionAttitude,
    MeasurementPositionVelocityAttitude,
    MeasurementRangeRateToPoint,
    MeasurementRangeToPoint,
    MeasurementSatnav,
    MeasurementSatnavSubframe,
    MeasurementSatnavWithSvData,
    MeasurementSpecificForce1D,
    MeasurementSpeed,
    MeasurementTemperature,
    MeasurementTime,
    MeasurementTimeDifference,
    MeasurementTimeFrequencyDifference,
    MeasurementVelocity,
]

LcmMsg: TypeAlias = Union[
    LcmMetadataBeidouEphemeris,
    LcmMetadataGlonassEphemeris,
    LcmMetadataGpsCnavEphemeris,
    LcmMetadataGpsLnavEphemeris,
    LcmMetadataGpsMnavEphemeris,
    LcmMetadataGpsIonoUtcParameters,
    LcmMetadataGalileoEphemeris,
    LcmMetadataImu,
    LcmMetadataGeneric,
    LcmMetadataImageFeatures,
    LcmMetadataMagneticField,
    LcmMetadataSatnavObs,
    LcmMeasurementImu,
    LcmMeasurementTdoa1Tx2Rx,
    LcmMeasurementTdoa2Tx1Rx,
    LcmMeasurementAccumulatedDistanceTraveled,
    LcmMeasurementAltitude,
    LcmMeasurementAngularVelocity,
    LcmMeasurementAngularVelocity1D,
    LcmMeasurementAttitude2D,
    LcmMeasurementAttitude3D,
    LcmMeasurementBarometer,
    LcmMeasurementDeltaPosition,
    LcmMeasurementDeltaRange,
    LcmMeasurementDeltaRangeToPoint,
    LcmMeasurementDirection2DToPoints,
    LcmMeasurementDirection3DToPoints,
    LcmMeasurementDirectionOfMotion2D,
    LcmMeasurementDirectionOfMotion3D,
    LcmMeasurementFrequencyDifference,
    LcmMeasurementHeading,
    LcmImage,
    LcmMeasurementMagneticField,
    LcmMeasurementMagneticFieldMagnitude,
    LcmMeasurementNavsimSatnavWithSvData,
    LcmMeasurementPosition,
    LcmMeasurementPositionAttitude,
    LcmMeasurementPositionVelocityAttitude,
    LcmMeasurementRangeRateToPoint,
    LcmMeasurementRangeToPoint,
    LcmMeasurementSatnav,
    LcmMeasurementSatnavSubframe,
    LcmMeasurementSatnavWithSvData,
    LcmMeasurementSpecificForce1D,
    LcmMeasurementSpeed,
    LcmMeasurementTemperature,
    LcmMeasurementTime,
    LcmMeasurementTimeDifference,
    LcmMeasurementTimeFrequencyDifference,
    LcmMeasurementVelocity,
]

to_lcm_map: dict[type[AspnMsg], Callable] = {
    MetadataBeidouEphemeris: metadata_BeiDou_ephemeris_to_lcm,
    MetadataGlonassEphemeris: metadata_GLONASS_ephemeris_to_lcm,
    MetadataGpsCnavEphemeris: metadata_GPS_Cnav_ephemeris_to_lcm,
    MetadataGpsLnavEphemeris: metadata_GPS_Lnav_ephemeris_to_lcm,
    MetadataGpsMnavEphemeris: metadata_GPS_Mnav_ephemeris_to_lcm,
    MetadataGpsIonoUtcParameters: metadata_GPS_iono_utc_parameters_to_lcm,
    MetadataGalileoEphemeris: metadata_Galileo_ephemeris_to_lcm,
    MetadataImu: metadata_IMU_to_lcm,
    MetadataGeneric: metadata_generic_to_lcm,
    MetadataImageFeatures: metadata_image_features_to_lcm,
    MetadataMagneticField: metadata_magnetic_field_to_lcm,
    MetadataSatnavObs: metadata_satnav_obs_to_lcm,
    MeasurementImu: measurement_IMU_to_lcm,
    MeasurementTdoa1Tx2Rx: measurement_TDOA_1Tx_2Rx_to_lcm,
    MeasurementTdoa2Tx1Rx: measurement_TDOA_2Tx_1Rx_to_lcm,
    MeasurementAccumulatedDistanceTraveled: measurement_accumulated_distance_traveled_to_lcm,
    MeasurementAltitude: measurement_altitude_to_lcm,
    MeasurementAngularVelocity: measurement_angular_velocity_to_lcm,
    MeasurementAngularVelocity1D: measurement_angular_velocity_1d_to_lcm,
    MeasurementAttitude2D: measurement_attitude_2d_to_lcm,
    MeasurementAttitude3D: measurement_attitude_3d_to_lcm,
    MeasurementBarometer: measurement_barometer_to_lcm,
    MeasurementDeltaPosition: measurement_delta_position_to_lcm,
    MeasurementDeltaRange: measurement_delta_range_to_lcm,
    MeasurementDeltaRangeToPoint: measurement_delta_range_to_point_to_lcm,
    MeasurementDirection2DToPoints: measurement_direction_2d_to_points_to_lcm,
    MeasurementDirection3DToPoints: measurement_direction_3d_to_points_to_lcm,
    MeasurementDirectionOfMotion2D: measurement_direction_of_motion_2d_to_lcm,
    MeasurementDirectionOfMotion3D: measurement_direction_of_motion_3d_to_lcm,
    MeasurementFrequencyDifference: measurement_frequency_difference_to_lcm,
    MeasurementHeading: measurement_heading_to_lcm,
    Image: image_to_lcm,
    MeasurementMagneticField: measurement_magnetic_field_to_lcm,
    MeasurementMagneticFieldMagnitude: measurement_magnetic_field_magnitude_to_lcm,
    MeasurementNavsimSatnavWithSvData: measurement_navsim_satnav_with_sv_data_to_lcm,
    MeasurementPosition: measurement_position_to_lcm,
    MeasurementPositionAttitude: measurement_position_attitude_to_lcm,
    MeasurementPositionVelocityAttitude: measurement_position_velocity_attitude_to_lcm,
    MeasurementRangeRateToPoint: measurement_range_rate_to_point_to_lcm,
    MeasurementRangeToPoint: measurement_range_to_point_to_lcm,
    MeasurementSatnav: measurement_satnav_to_lcm,
    MeasurementSatnavSubframe: measurement_satnav_subframe_to_lcm,
    MeasurementSatnavWithSvData: measurement_satnav_with_sv_data_to_lcm,
    MeasurementSpecificForce1D: measurement_specific_force_1d_to_lcm,
    MeasurementSpeed: measurement_speed_to_lcm,
    MeasurementTemperature: measurement_temperature_to_lcm,
    MeasurementTime: measurement_time_to_lcm,
    MeasurementTimeDifference: measurement_time_difference_to_lcm,
    MeasurementTimeFrequencyDifference: measurement_time_frequency_difference_to_lcm,
    MeasurementVelocity: measurement_velocity_to_lcm,
}

from_lcm_map: dict[type[LcmMsg], Callable] = {
    LcmMetadataBeidouEphemeris: lcm_to_metadata_BeiDou_ephemeris,
    LcmMetadataGlonassEphemeris: lcm_to_metadata_GLONASS_ephemeris,
    LcmMetadataGpsCnavEphemeris: lcm_to_metadata_GPS_Cnav_ephemeris,
    LcmMetadataGpsLnavEphemeris: lcm_to_metadata_GPS_Lnav_ephemeris,
    LcmMetadataGpsMnavEphemeris: lcm_to_metadata_GPS_Mnav_ephemeris,
    LcmMetadataGpsIonoUtcParameters: lcm_to_metadata_GPS_iono_utc_parameters,
    LcmMetadataGalileoEphemeris: lcm_to_metadata_Galileo_ephemeris,
    LcmMetadataImu: lcm_to_metadata_IMU,
    LcmMetadataGeneric: lcm_to_metadata_generic,
    LcmMetadataImageFeatures: lcm_to_metadata_image_features,
    LcmMetadataMagneticField: lcm_to_metadata_magnetic_field,
    LcmMetadataSatnavObs: lcm_to_metadata_satnav_obs,
    LcmMeasurementImu: lcm_to_measurement_IMU,
    LcmMeasurementTdoa1Tx2Rx: lcm_to_measurement_TDOA_1Tx_2Rx,
    LcmMeasurementTdoa2Tx1Rx: lcm_to_measurement_TDOA_2Tx_1Rx,
    LcmMeasurementAccumulatedDistanceTraveled: lcm_to_measurement_accumulated_distance_traveled,
    LcmMeasurementAltitude: lcm_to_measurement_altitude,
    LcmMeasurementAngularVelocity: lcm_to_measurement_angular_velocity,
    LcmMeasurementAngularVelocity1D: lcm_to_measurement_angular_velocity_1d,
    LcmMeasurementAttitude2D: lcm_to_measurement_attitude_2d,
    LcmMeasurementAttitude3D: lcm_to_measurement_attitude_3d,
    LcmMeasurementBarometer: lcm_to_measurement_barometer,
    LcmMeasurementDeltaPosition: lcm_to_measurement_delta_position,
    LcmMeasurementDeltaRange: lcm_to_measurement_delta_range,
    LcmMeasurementDeltaRangeToPoint: lcm_to_measurement_delta_range_to_point,
    LcmMeasurementDirection2DToPoints: lcm_to_measurement_direction_2d_to_points,
    LcmMeasurementDirection3DToPoints: lcm_to_measurement_direction_3d_to_points,
    LcmMeasurementDirectionOfMotion2D: lcm_to_measurement_direction_of_motion_2d,
    LcmMeasurementDirectionOfMotion3D: lcm_to_measurement_direction_of_motion_3d,
    LcmMeasurementFrequencyDifference: lcm_to_measurement_frequency_difference,
    LcmMeasurementHeading: lcm_to_measurement_heading,
    LcmImage: lcm_to_image,
    LcmMeasurementMagneticField: lcm_to_measurement_magnetic_field,
    LcmMeasurementMagneticFieldMagnitude: lcm_to_measurement_magnetic_field_magnitude,
    LcmMeasurementNavsimSatnavWithSvData: lcm_to_measurement_navsim_satnav_with_sv_data,
    LcmMeasurementPosition: lcm_to_measurement_position,
    LcmMeasurementPositionAttitude: lcm_to_measurement_position_attitude,
    LcmMeasurementPositionVelocityAttitude: lcm_to_measurement_position_velocity_attitude,
    LcmMeasurementRangeRateToPoint: lcm_to_measurement_range_rate_to_point,
    LcmMeasurementRangeToPoint: lcm_to_measurement_range_to_point,
    LcmMeasurementSatnav: lcm_to_measurement_satnav,
    LcmMeasurementSatnavSubframe: lcm_to_measurement_satnav_subframe,
    LcmMeasurementSatnavWithSvData: lcm_to_measurement_satnav_with_sv_data,
    LcmMeasurementSpecificForce1D: lcm_to_measurement_specific_force_1d,
    LcmMeasurementSpeed: lcm_to_measurement_speed,
    LcmMeasurementTemperature: lcm_to_measurement_temperature,
    LcmMeasurementTime: lcm_to_measurement_time,
    LcmMeasurementTimeDifference: lcm_to_measurement_time_difference,
    LcmMeasurementTimeFrequencyDifference: lcm_to_measurement_time_frequency_difference,
    LcmMeasurementVelocity: lcm_to_measurement_velocity,
}

decode_lcm_map: dict[bytes, Callable] = {
    LcmMetadataBeidouEphemeris._get_packed_fingerprint(): LcmMetadataBeidouEphemeris.decode,
    LcmMetadataGlonassEphemeris._get_packed_fingerprint(): LcmMetadataGlonassEphemeris.decode,
    LcmMetadataGpsCnavEphemeris._get_packed_fingerprint(): LcmMetadataGpsCnavEphemeris.decode,
    LcmMetadataGpsLnavEphemeris._get_packed_fingerprint(): LcmMetadataGpsLnavEphemeris.decode,
    LcmMetadataGpsMnavEphemeris._get_packed_fingerprint(): LcmMetadataGpsMnavEphemeris.decode,
    LcmMetadataGpsIonoUtcParameters._get_packed_fingerprint(): LcmMetadataGpsIonoUtcParameters.decode,
    LcmMetadataGalileoEphemeris._get_packed_fingerprint(): LcmMetadataGalileoEphemeris.decode,
    LcmMetadataImu._get_packed_fingerprint(): LcmMetadataImu.decode,
    LcmMetadataGeneric._get_packed_fingerprint(): LcmMetadataGeneric.decode,
    LcmMetadataImageFeatures._get_packed_fingerprint(): LcmMetadataImageFeatures.decode,
    LcmMetadataMagneticField._get_packed_fingerprint(): LcmMetadataMagneticField.decode,
    LcmMetadataSatnavObs._get_packed_fingerprint(): LcmMetadataSatnavObs.decode,
    LcmMeasurementImu._get_packed_fingerprint(): LcmMeasurementImu.decode,
    LcmMeasurementTdoa1Tx2Rx._get_packed_fingerprint(): LcmMeasurementTdoa1Tx2Rx.decode,
    LcmMeasurementTdoa2Tx1Rx._get_packed_fingerprint(): LcmMeasurementTdoa2Tx1Rx.decode,
    LcmMeasurementAccumulatedDistanceTraveled._get_packed_fingerprint(): LcmMeasurementAccumulatedDistanceTraveled.decode,
    LcmMeasurementAltitude._get_packed_fingerprint(): LcmMeasurementAltitude.decode,
    LcmMeasurementAngularVelocity._get_packed_fingerprint(): LcmMeasurementAngularVelocity.decode,
    LcmMeasurementAngularVelocity1D._get_packed_fingerprint(): LcmMeasurementAngularVelocity1D.decode,
    LcmMeasurementAttitude2D._get_packed_fingerprint(): LcmMeasurementAttitude2D.decode,
    LcmMeasurementAttitude3D._get_packed_fingerprint(): LcmMeasurementAttitude3D.decode,
    LcmMeasurementBarometer._get_packed_fingerprint(): LcmMeasurementBarometer.decode,
    LcmMeasurementDeltaPosition._get_packed_fingerprint(): LcmMeasurementDeltaPosition.decode,
    LcmMeasurementDeltaRange._get_packed_fingerprint(): LcmMeasurementDeltaRange.decode,
    LcmMeasurementDeltaRangeToPoint._get_packed_fingerprint(): LcmMeasurementDeltaRangeToPoint.decode,
    LcmMeasurementDirection2DToPoints._get_packed_fingerprint(): LcmMeasurementDirection2DToPoints.decode,
    LcmMeasurementDirection3DToPoints._get_packed_fingerprint(): LcmMeasurementDirection3DToPoints.decode,
    LcmMeasurementDirectionOfMotion2D._get_packed_fingerprint(): LcmMeasurementDirectionOfMotion2D.decode,
    LcmMeasurementDirectionOfMotion3D._get_packed_fingerprint(): LcmMeasurementDirectionOfMotion3D.decode,
    LcmMeasurementFrequencyDifference._get_packed_fingerprint(): LcmMeasurementFrequencyDifference.decode,
    LcmMeasurementHeading._get_packed_fingerprint(): LcmMeasurementHeading.decode,
    LcmImage._get_packed_fingerprint(): LcmImage.decode,
    LcmMeasurementMagneticField._get_packed_fingerprint(): LcmMeasurementMagneticField.decode,
    LcmMeasurementMagneticFieldMagnitude._get_packed_fingerprint(): LcmMeasurementMagneticFieldMagnitude.decode,
    LcmMeasurementNavsimSatnavWithSvData._get_packed_fingerprint(): LcmMeasurementNavsimSatnavWithSvData.decode,
    LcmMeasurementPosition._get_packed_fingerprint(): LcmMeasurementPosition.decode,
    LcmMeasurementPositionAttitude._get_packed_fingerprint(): LcmMeasurementPositionAttitude.decode,
    LcmMeasurementPositionVelocityAttitude._get_packed_fingerprint(): LcmMeasurementPositionVelocityAttitude.decode,
    LcmMeasurementRangeRateToPoint._get_packed_fingerprint(): LcmMeasurementRangeRateToPoint.decode,
    LcmMeasurementRangeToPoint._get_packed_fingerprint(): LcmMeasurementRangeToPoint.decode,
    LcmMeasurementSatnav._get_packed_fingerprint(): LcmMeasurementSatnav.decode,
    LcmMeasurementSatnavSubframe._get_packed_fingerprint(): LcmMeasurementSatnavSubframe.decode,
    LcmMeasurementSatnavWithSvData._get_packed_fingerprint(): LcmMeasurementSatnavWithSvData.decode,
    LcmMeasurementSpecificForce1D._get_packed_fingerprint(): LcmMeasurementSpecificForce1D.decode,
    LcmMeasurementSpeed._get_packed_fingerprint(): LcmMeasurementSpeed.decode,
    LcmMeasurementTemperature._get_packed_fingerprint(): LcmMeasurementTemperature.decode,
    LcmMeasurementTime._get_packed_fingerprint(): LcmMeasurementTime.decode,
    LcmMeasurementTimeDifference._get_packed_fingerprint(): LcmMeasurementTimeDifference.decode,
    LcmMeasurementTimeFrequencyDifference._get_packed_fingerprint(): LcmMeasurementTimeFrequencyDifference.decode,
    LcmMeasurementVelocity._get_packed_fingerprint(): LcmMeasurementVelocity.decode,
}
