import javax.swing.*;
import java.awt.*;
import java.awt.image.*;

public class MapWindow extends JFrame {

    private BufferedImage theMap;
    private int imwidth, imheight;
    private double scale; // meters per pixel
    private MapPanel mp;
    private double[][] evidenceGrid;
    
    public MapWindow(int width, int height, double scale) {
        evidenceGrid = new double[width][height];

	this.scale = scale;
        imwidth = width;
        imheight = height;
        theMap = new BufferedImage(imwidth,imheight,
                     BufferedImage.TYPE_INT_ARGB);
        int midgray = (0xff << 24) | (128 << 16) | (128 << 8) | (128);
        
	for (int x = 0; x < imwidth; x++)
            for (int y = 0; y < imheight; y++)
            {
	    	theMap.setRGB(x,y,midgray);
		evidenceGrid[x][y] = 1;
	    }
        	
	mp = new MapPanel();
        add(mp);
        pack();
        setDefaultCloseOperation(EXIT_ON_CLOSE);
        setLocationRelativeTo(null);
        setVisible(true);
    }

    void setPixel(int imx, int imy, int value) {
        if (value < 0 || value > 255)
            return;
        int rgbval = (0xff << 24) | (value << 16) | (value << 8) | value;
        if (imx >= 0 && imx < imwidth && imy >= 0 && imy < imheight)
            theMap.setRGB(imx,imy,rgbval);
    }

    void colorPixel(int imx, int imy, Color c) {
        theMap.setRGB(imx,imy,c.getRGB());
    }

    void setXY(double x, double y, double value) {
        int xpix = (int)(((x/100) / scale) + imwidth/2);
        int ypix = (int)(imheight/2 - ((y/100) / scale));
        setPixel(xpix, ypix, (int)(value*255));
    }

    void newEvidence(int r, int c, double odds)
    {
	evidenceGrid[r][c] *= odds;
    }
    
    void reColor()
    {
	for(int r = 0; r < imheight; r++)
		for(int c = 0; c < imwidth; c++)
		{
			double probs = evidenceGrid[r][c] / (1 + evidenceGrid[r][c]);
			int obstacleness = (int)(255 * probs);
			setPixel(c, r, obstacleness);
		}
	touch();
    }

    void touch() { mp.repaint(); }

    class MapPanel extends JPanel {

        protected void paintComponent(Graphics g) {
            g.drawImage(theMap,0,0,null);
        }
        public Dimension getPreferredSize() {
            return new Dimension(imwidth,imheight);
        }
    }
}
