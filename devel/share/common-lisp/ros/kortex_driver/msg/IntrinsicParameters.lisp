; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude IntrinsicParameters.msg.html

(cl:defclass <IntrinsicParameters> (roslisp-msg-protocol:ros-message)
  ((sensor
    :reader sensor
    :initarg :sensor
    :type cl:integer
    :initform 0)
   (resolution
    :reader resolution
    :initarg :resolution
    :type cl:integer
    :initform 0)
   (principal_point_x
    :reader principal_point_x
    :initarg :principal_point_x
    :type cl:float
    :initform 0.0)
   (principal_point_y
    :reader principal_point_y
    :initarg :principal_point_y
    :type cl:float
    :initform 0.0)
   (focal_length_x
    :reader focal_length_x
    :initarg :focal_length_x
    :type cl:float
    :initform 0.0)
   (focal_length_y
    :reader focal_length_y
    :initarg :focal_length_y
    :type cl:float
    :initform 0.0)
   (distortion_coeffs
    :reader distortion_coeffs
    :initarg :distortion_coeffs
    :type kortex_driver-msg:DistortionCoefficients
    :initform (cl:make-instance 'kortex_driver-msg:DistortionCoefficients)))
)

(cl:defclass IntrinsicParameters (<IntrinsicParameters>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IntrinsicParameters>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IntrinsicParameters)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<IntrinsicParameters> is deprecated: use kortex_driver-msg:IntrinsicParameters instead.")))

(cl:ensure-generic-function 'sensor-val :lambda-list '(m))
(cl:defmethod sensor-val ((m <IntrinsicParameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:sensor-val is deprecated.  Use kortex_driver-msg:sensor instead.")
  (sensor m))

(cl:ensure-generic-function 'resolution-val :lambda-list '(m))
(cl:defmethod resolution-val ((m <IntrinsicParameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:resolution-val is deprecated.  Use kortex_driver-msg:resolution instead.")
  (resolution m))

(cl:ensure-generic-function 'principal_point_x-val :lambda-list '(m))
(cl:defmethod principal_point_x-val ((m <IntrinsicParameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:principal_point_x-val is deprecated.  Use kortex_driver-msg:principal_point_x instead.")
  (principal_point_x m))

(cl:ensure-generic-function 'principal_point_y-val :lambda-list '(m))
(cl:defmethod principal_point_y-val ((m <IntrinsicParameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:principal_point_y-val is deprecated.  Use kortex_driver-msg:principal_point_y instead.")
  (principal_point_y m))

(cl:ensure-generic-function 'focal_length_x-val :lambda-list '(m))
(cl:defmethod focal_length_x-val ((m <IntrinsicParameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:focal_length_x-val is deprecated.  Use kortex_driver-msg:focal_length_x instead.")
  (focal_length_x m))

(cl:ensure-generic-function 'focal_length_y-val :lambda-list '(m))
(cl:defmethod focal_length_y-val ((m <IntrinsicParameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:focal_length_y-val is deprecated.  Use kortex_driver-msg:focal_length_y instead.")
  (focal_length_y m))

(cl:ensure-generic-function 'distortion_coeffs-val :lambda-list '(m))
(cl:defmethod distortion_coeffs-val ((m <IntrinsicParameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:distortion_coeffs-val is deprecated.  Use kortex_driver-msg:distortion_coeffs instead.")
  (distortion_coeffs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IntrinsicParameters>) ostream)
  "Serializes a message object of type '<IntrinsicParameters>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'resolution)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'resolution)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'resolution)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'resolution)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'principal_point_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'principal_point_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'focal_length_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'focal_length_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'distortion_coeffs) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IntrinsicParameters>) istream)
  "Deserializes a message object of type '<IntrinsicParameters>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'resolution)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'resolution)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'resolution)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'resolution)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'principal_point_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'principal_point_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'focal_length_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'focal_length_y) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'distortion_coeffs) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IntrinsicParameters>)))
  "Returns string type for a message object of type '<IntrinsicParameters>"
  "kortex_driver/IntrinsicParameters")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IntrinsicParameters)))
  "Returns string type for a message object of type 'IntrinsicParameters"
  "kortex_driver/IntrinsicParameters")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IntrinsicParameters>)))
  "Returns md5sum for a message object of type '<IntrinsicParameters>"
  "23b95f71707877a1e06e2ac8ba309bb9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IntrinsicParameters)))
  "Returns md5sum for a message object of type 'IntrinsicParameters"
  "23b95f71707877a1e06e2ac8ba309bb9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IntrinsicParameters>)))
  "Returns full string definition for message of type '<IntrinsicParameters>"
  (cl:format cl:nil "~%uint32 sensor~%uint32 resolution~%float32 principal_point_x~%float32 principal_point_y~%float32 focal_length_x~%float32 focal_length_y~%DistortionCoefficients distortion_coeffs~%================================================================================~%MSG: kortex_driver/DistortionCoefficients~%~%float32 k1~%float32 k2~%float32 k3~%float32 p1~%float32 p2~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IntrinsicParameters)))
  "Returns full string definition for message of type 'IntrinsicParameters"
  (cl:format cl:nil "~%uint32 sensor~%uint32 resolution~%float32 principal_point_x~%float32 principal_point_y~%float32 focal_length_x~%float32 focal_length_y~%DistortionCoefficients distortion_coeffs~%================================================================================~%MSG: kortex_driver/DistortionCoefficients~%~%float32 k1~%float32 k2~%float32 k3~%float32 p1~%float32 p2~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IntrinsicParameters>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'distortion_coeffs))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IntrinsicParameters>))
  "Converts a ROS message object to a list"
  (cl:list 'IntrinsicParameters
    (cl:cons ':sensor (sensor msg))
    (cl:cons ':resolution (resolution msg))
    (cl:cons ':principal_point_x (principal_point_x msg))
    (cl:cons ':principal_point_y (principal_point_y msg))
    (cl:cons ':focal_length_x (focal_length_x msg))
    (cl:cons ':focal_length_y (focal_length_y msg))
    (cl:cons ':distortion_coeffs (distortion_coeffs msg))
))
