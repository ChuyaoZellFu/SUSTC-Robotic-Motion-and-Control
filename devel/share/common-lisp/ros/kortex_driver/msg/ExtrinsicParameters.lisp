; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ExtrinsicParameters.msg.html

(cl:defclass <ExtrinsicParameters> (roslisp-msg-protocol:ros-message)
  ((rotation
    :reader rotation
    :initarg :rotation
    :type kortex_driver-msg:VisionConfig_RotationMatrix
    :initform (cl:make-instance 'kortex_driver-msg:VisionConfig_RotationMatrix))
   (translation
    :reader translation
    :initarg :translation
    :type kortex_driver-msg:TranslationVector
    :initform (cl:make-instance 'kortex_driver-msg:TranslationVector)))
)

(cl:defclass ExtrinsicParameters (<ExtrinsicParameters>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExtrinsicParameters>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExtrinsicParameters)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ExtrinsicParameters> is deprecated: use kortex_driver-msg:ExtrinsicParameters instead.")))

(cl:ensure-generic-function 'rotation-val :lambda-list '(m))
(cl:defmethod rotation-val ((m <ExtrinsicParameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:rotation-val is deprecated.  Use kortex_driver-msg:rotation instead.")
  (rotation m))

(cl:ensure-generic-function 'translation-val :lambda-list '(m))
(cl:defmethod translation-val ((m <ExtrinsicParameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:translation-val is deprecated.  Use kortex_driver-msg:translation instead.")
  (translation m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExtrinsicParameters>) ostream)
  "Serializes a message object of type '<ExtrinsicParameters>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'rotation) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'translation) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExtrinsicParameters>) istream)
  "Deserializes a message object of type '<ExtrinsicParameters>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'rotation) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'translation) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExtrinsicParameters>)))
  "Returns string type for a message object of type '<ExtrinsicParameters>"
  "kortex_driver/ExtrinsicParameters")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExtrinsicParameters)))
  "Returns string type for a message object of type 'ExtrinsicParameters"
  "kortex_driver/ExtrinsicParameters")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExtrinsicParameters>)))
  "Returns md5sum for a message object of type '<ExtrinsicParameters>"
  "931ae3c2dcff1b69750ed6505e17f9d8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExtrinsicParameters)))
  "Returns md5sum for a message object of type 'ExtrinsicParameters"
  "931ae3c2dcff1b69750ed6505e17f9d8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExtrinsicParameters>)))
  "Returns full string definition for message of type '<ExtrinsicParameters>"
  (cl:format cl:nil "~%VisionConfig_RotationMatrix rotation~%TranslationVector translation~%================================================================================~%MSG: kortex_driver/VisionConfig_RotationMatrix~%~%VisionConfig_RotationMatrixRow row1~%VisionConfig_RotationMatrixRow row2~%VisionConfig_RotationMatrixRow row3~%================================================================================~%MSG: kortex_driver/VisionConfig_RotationMatrixRow~%~%float32 column1~%float32 column2~%float32 column3~%================================================================================~%MSG: kortex_driver/TranslationVector~%~%float32 t_x~%float32 t_y~%float32 t_z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExtrinsicParameters)))
  "Returns full string definition for message of type 'ExtrinsicParameters"
  (cl:format cl:nil "~%VisionConfig_RotationMatrix rotation~%TranslationVector translation~%================================================================================~%MSG: kortex_driver/VisionConfig_RotationMatrix~%~%VisionConfig_RotationMatrixRow row1~%VisionConfig_RotationMatrixRow row2~%VisionConfig_RotationMatrixRow row3~%================================================================================~%MSG: kortex_driver/VisionConfig_RotationMatrixRow~%~%float32 column1~%float32 column2~%float32 column3~%================================================================================~%MSG: kortex_driver/TranslationVector~%~%float32 t_x~%float32 t_y~%float32 t_z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExtrinsicParameters>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'rotation))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'translation))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExtrinsicParameters>))
  "Converts a ROS message object to a list"
  (cl:list 'ExtrinsicParameters
    (cl:cons ':rotation (rotation msg))
    (cl:cons ':translation (translation msg))
))
