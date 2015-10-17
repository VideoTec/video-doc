private final byte[] AMR_HEADER = new byte[] { 0x23, 0x21, 0x41, 0x4d, 0x52, 0x0a };

    private String addAMRHeader(String audioFile) {
        String result = "";
        String suffix = "-with-header.amr";
        FileOutputStream out = null;
        InputStream in = null;
        try {
            int len = audioFile.length();
            if (audioFile.endsWith(".amr")) {
                result = audioFile.substring(0, len - 4) + suffix;
            } else {
                result = audioFile + suffix;
            }
            LogSystem.v(TAG, "local audio file: " + result);
            File outFile = new File(result);
            if (outFile.exists()) {
                return result;
            }
            
            out = new FileOutputStream(outFile);
            in = new BufferedInputStream(new FileInputStream(audioFile));
            out.write(AMR_HEADER);
            byte[] buffer = new byte[1024];

            while ((len = in.read(buffer)) != -1) {
                out.write(buffer, 0, len);
            }
        } catch (Exception ex) {
            return "";
        } finally {
            if (in != null) {
                try {
                    in.close();
                } catch (Exception ex) {
                    ex.printStackTrace();
                }
            }
            if (out != null) {
                try {
                    out.close();
                } catch (Exception ex) {
                    ex.printStackTrace();
                }
            }
        }
        return result;
    }