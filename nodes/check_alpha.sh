rostopic echo /psp_ally1 |& tee /tmp/psp_ally1 & \
rostopic echo /psp_ally1_estimator |& tee /tmp/psp_ally1_estimator & \
sleep 5 && \
kill %1 && \
kill %2
