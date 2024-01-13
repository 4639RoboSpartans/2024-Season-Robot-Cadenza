package math;

public record vec2 (double x, double y) {
    public static final vec2 iHat = new vec2(1, 0);
    public static final vec2 jHat = new vec2(0, 1);

    public vec2 plus(vec2 v){
        return new vec2(x() + v.x(), y() + v.y());
    }

    public vec2 minus(vec2 v){
        return new vec2(x() - v.x(), y() - v.y());
    }

    public double dot(vec2 v){
        return x() * v.x() + y() * v.y();
    }

    public vec2 times(double scale){
        return new vec2(x() * scale, y() * scale);
    }

    public String toString(){
        return "("
                + math.round(x, 0.0001)
                + ", "
                + math.round(y, 0.0001)
                + ")";
    }
}