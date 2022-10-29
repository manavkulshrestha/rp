import tensorflow as tf
import tensorflow_probability as tfp
tfpd = tfp.distributions

d = tfpd.JointDistributionSequential([
	tfpd.Uniform(),
	tfpd.Uniform(),
	tfpd.Uniform()
])

print(d.resolve_graph())