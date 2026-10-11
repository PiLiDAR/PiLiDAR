import numpy as np
import cv2

def color_transfer(source, target, lab=True):
	"""
	Transfers the color distribution from the source to the target
	image using the mean and standard deviations of the L*a*b*
	color space.

	This implementation is (loosely) based on to the "Color Transfer
	between Images" paper by Reinhard et al., 2001.
	"""
	if lab:
		source = cv2.cvtColor(source, cv2.COLOR_BGR2LAB).astype("float32")
		target = cv2.cvtColor(target, cv2.COLOR_BGR2LAB).astype("float32")
	else:
		source = source.astype("float32")
		target = target.astype("float32")

	# compute color statistics for the source and target images
	(bMeanSrc, bStdSrc, gMeanSrc, gStdSrc, rMeanSrc, rStdSrc) = image_stats(source)
	(bMeanTar, bStdTar, gMeanTar, gStdTar, rMeanTar, rStdTar) = image_stats(target)

	print(bMeanSrc, bMeanTar, bMeanTar/bMeanSrc)
	print(gMeanSrc, gMeanTar, gMeanTar/gMeanSrc)
	print(rMeanSrc, rMeanTar, rMeanTar/rMeanSrc)

	# subtract the means from the target image
	(b, g, r) = cv2.split(target)
	b -= bMeanTar
	g -= gMeanTar
	r -= rMeanTar

	# scale by the standard deviations
	b *= (bStdTar / bStdSrc)
	g *= (gStdTar / gStdSrc)
	r *= (rStdTar / rStdSrc)

	# add in the source mean
	b += bMeanSrc
	g += gMeanSrc
	r += rMeanSrc

	b = np.clip(b, 0, 255)
	g = np.clip(g, 0, 255)
	r = np.clip(r, 0, 255)

	transfer = cv2.merge([b, g, r])
	
	if lab:
		transfer = cv2.cvtColor(transfer.astype("uint8"), cv2.COLOR_LAB2BGR)
	else:
		transfer = transfer.astype("uint8")

	return transfer


def image_stats(image):
	(b, g, r) = cv2.split(image)
	return b.mean(), b.std(), g.mean(), g.std(), r.mean(), r.std()


if __name__ == "__main__":
	# source = cv2.imread("images/tmp/img0.jpg")
	# target = cv2.imread("images/tmp/awb.jpg")
	# transfer = color_transfer(source, target, lab=False)
	# # cv2.imwrite("export/color_transfer.jpg", transfer)
	# cv2.imshow("Source", source)
	# cv2.imshow("Target", target)
	# cv2.imshow("Transfer", transfer)
	# cv2.waitKey(0)


	# JUST CALCULATE AWB GAINS

	def awbgains(img1, img2):  # source, target
		(b1, _, r1) = cv2.split(img1.astype("float32"))
		(b2, _, r2) = cv2.split(img2.astype("float32"))

		R = r2.mean() / r1.mean()
		B = b2.mean() / b1.mean()
		return R, B
		
	img1 = cv2.imread("images/tmp/img0.jpg")
	img2 = cv2.imread("images/tmp/awb.jpg")
	R, B = awbgains(img1, img2)
	print(R, B)
