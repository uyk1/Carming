import {
  TouchableOpacity,
  Text,
  ViewStyle,
  StyleSheet,
  TextStyle,
} from 'react-native'; // 리액트 네이티브에서 제공하는 컴포넌트 추가

interface CustomButtonProps {
  text: string;
  buttonStyle?: ViewStyle;
  textStyle?: TextStyle;
  onPress?: () => void;
  disabled?: boolean;
}

const CustomButton: React.FC<CustomButtonProps> = ({
  text,
  buttonStyle,
  textStyle,
  onPress,
  disabled = false,
}) => {
  const handlePress = () => {
    if (onPress && !disabled) {
      onPress();
    }
  };

  return (
    <TouchableOpacity
      style={[styles.button, buttonStyle]}
      onPress={handlePress}
      disabled={disabled}>
      <Text style={[styles.text, textStyle]}> {text} </Text>
    </TouchableOpacity>
  );
};

const styles = StyleSheet.create({
  button: {
    backgroundColor: 'black',
    padding: 16,
    margin: 10,
    borderRadius: 8,
  },
  text: {
    color: 'white',
    fontSize: 24,
  },
});
export default CustomButton;
